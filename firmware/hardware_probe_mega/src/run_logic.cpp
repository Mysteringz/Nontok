// Port of tools/seek_steps.py (the version run on the field on 30 Oct). Comments there explain
// each rule and the field observation behind it; the numbers here must stay in step with it.
#include "run_logic.h"
#include <math.h>
#include <string.h>

namespace rl {

#define TRY(x) do { if (!(x)) return false; } while (0)

static const uint8_t PWM_X = 85, PWM_Y = 100, ROT_PWM = 120, ROT_FINE_PWM = 80;
static const float HOLD_TOL = 4;        // deg: the gentlest rotation pulse still turns ~7 deg
static const float FRACTION = 0.6f;     // cover this share of the error per move: undershoot
static const float DEAD_MS = 60;        // soft start: the first ~60 ms barely moves the robot
static const uint16_t T_MIN = 120, T_MAX = 300;   // gyro FIFO holds ~0.85 s; long moves veer
static const int16_t SIDE_WINDOW = 250; // a side reading this far off its target is another surface

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static int16_t i16(float v) { return int16_t(clampf(v, -32000, 32000)); }

Runner::Runner(Io& io_, const Calib& cal_)
    : io(io_), cal(cal_), why(0), goal(0), goalSet(false), goalLost(0), turnDir(0) {
  k[0] = 0.35f;
  k[1] = 0.25f;
  memset(wcomp, 0, sizeof(wcomp));
  rate[0] = rate[1] = 0;
}

bool Runner::measure(int16_t us[NUS], float* rg) {
  float dummy;
  if (!io.measure(us, rg ? rg : &dummy)) return fail(RL_F("stopped"));
  return true;
}

bool Runner::pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, float ms, uint16_t* used) {
  uint16_t m = uint16_t(clampf(ms, T_MIN, T_MAX));
  if (used) *used = m;
  if (!io.pulse(x, y, w, pwm, m)) return fail(RL_F("stopped"));
  return true;
}

bool Runner::setGoal(float g) {
  goal = g;
  goalSet = true;
  goalLost = io.lost();
  return true;
}

// ex (+ = drive forward), ey (+ = strafe left); has* false = not measurable.
bool Runner::errors(Pose pose, const int16_t us[NUS], bool creep, float* ex, bool* hasEx, float* ey, bool* hasEy) {
  const int16_t* ref = cal.ref[pose];
  #define HAS(i) (ref[i] != NONE)
  #define OK(i) (us[i] > 0)
  #define NEAR(i) (HAS(i) && OK(i) && fabsf(float(us[i] - ref[i])) < SIDE_WINDOW)
  *hasEx = HAS(FRONT) && OK(FRONT);
  *ex = float(us[FRONT] - ref[FRONT]);
  *hasEy = false;
  if (HAS(LSIDE) && HAS(RSIDE)) {
    // Both side walls, proven by their spacing: trust them however far off the robot is.
    bool spaced = OK(LSIDE) && OK(RSIDE) &&
                  fabsf(float((us[LSIDE] + us[RSIDE]) - (ref[LSIDE] + ref[RSIDE]))) <= 100;
    if (spaced || (NEAR(LSIDE) && NEAR(RSIDE))) {
      *ey = ((us[LSIDE] - us[RSIDE]) - (ref[LSIDE] - ref[RSIDE])) / 2.0f;
      *hasEy = true;
    }
  } else if (HAS(RSIDE)) {
    if (NEAR(RSIDE)) { *ey = float(ref[RSIDE] - us[RSIDE]); *hasEy = true; }
  } else if (HAS(LSIDE)) {
    if (NEAR(LSIDE)) { *ey = float(us[LSIDE] - ref[LSIDE]); *hasEy = true; }
  }
  if (!*hasEy && creep && HAS(L45) && HAS(R45) && OK(L45) && OK(R45)) {
    *ey = ((us[L45] - us[R45]) - (ref[L45] - ref[R45])) * 0.354f;
    *hasEy = true;
  }
  #undef HAS
  #undef OK
  #undef NEAR
  return true;
}

// One move along x (axis 0) or y (axis 1), then fresh readings. A move also turns this robot
// (strafes CCW, forward CW): each axis and sign learns how much opposite rotation to mix in.
bool Runner::translate(uint8_t axis, float err, int16_t us[NUS], float maxMs, uint16_t* used) {
  float ms = DEAD_MS + FRACTION * fabsf(err) / k[axis];
  if (ms > maxMs) ms = maxMs;
  if (axis == 1 && ms > 200) ms = 200;   // a 300 ms strafe also rotated the robot
  int8_t sign = err > 0 ? 1 : -1;
  float before, after;
  bool hasBefore = io.yaw(&before);
  int8_t w = wcomp[axis][sign > 0];
  TRY(pulse(axis == 0 ? sign * 100 : 0, axis == 1 ? sign * 100 : 0, w, axis == 0 ? PWM_X : PWM_Y, ms, used));
  TRY(measure(us));
  if (hasBefore && io.yaw(&after)) {
    float gain = axis == 1 ? 2.0f : 1.5f;
    wcomp[axis][sign > 0] = int8_t(clampf(roundf(w - gain * (after - before)), -40, 40));
  }
  return true;
}

bool Runner::learn(uint8_t axis, bool hasBefore, float before, bool hasAfter, float after, uint16_t ms) {
  if (!hasBefore || !hasAfter) return true;
  float moved = before * after > 0 ? fabsf(before) - fabsf(after) : fabsf(before) + fabsf(after);
  if (moved < -25) {
    if (axis == 0) return fail(RL_F("forward_move_went_the_wrong_way"));
    return true;   // sideways from the jumpy 45 deg beams: a bad reading, not a reason to stop
  }
  if (moved > 5) {
    float kk = moved / (ms - DEAD_MS > 10 ? ms - DEAD_MS : 10);
    float nw = 0.5f * k[axis] + 0.5f * kk;
    k[axis] = clampf(clampf(nw, k[axis] / 1.5f, k[axis] * 1.5f), 0.05f, 1.5f);
  } else {
    k[axis] = k[axis] * 0.6f < 0.05f ? 0.05f : k[axis] * 0.6f;   // did not move: longer next time
  }
  return true;
}

// Keep the gyro heading: back to the goal if a move turned the robot more than HOLD_TOL.
bool Runner::hold(int16_t us[NUS]) {
  float y;
  if (!io.yaw(&y)) return true;
  if (!goalSet || io.lost() != goalLost) {
    if (goalSet) io.event(RL_F("gyro_rebase"), i16(y * 10));
    setGoal(y);
  }
  if (fabsf(goal - y) <= HOLD_TOL) return true;
  io.event(RL_F("hold"), i16(y * 10), i16(goal * 10));
  TRY(rotateTo(goal, HOLD_TOL, true));
  return measure(us);
}

// Gyro rotation, read only while stopped. Each pulse aims at 80 % of the remaining angle from
// the rate measured at that power; under 12 deg the gentler power. A pulse that barely turns
// makes the next one longer; an overshoot is corrected by the next pulse back. strict = must
// reach tol; otherwise best effort, stopping once pulses no longer get closer.
bool Runner::rotateTo(float target, float tol, bool strict, float* reached) {
  uint16_t startMs[2] = {200, T_MIN};   // untried rate: big pulse ~30 deg, fine pulse shortest
  float best = -1;
  uint8_t stalls = 0;
  uint16_t lost0 = io.lost();
  int16_t us[NUS];
  for (uint8_t move = 0; move < (strict ? 25 : 6); ++move) {
    float y;
    if (!io.yaw(&y) || io.lost() != lost0) return fail(RL_F("gyro_lost_data_in_rotation"));
    float err = target - y;
    if (fabsf(err) <= tol) { if (reached) *reached = y; return true; }
    if (!strict) {
      stalls = (best >= 0 && fabsf(err) >= best - 0.5f) ? stalls + 1 : 0;
      best = best < 0 ? fabsf(err) : (fabsf(err) < best ? fabsf(err) : best);
      if (stalls >= 2) { if (reached) *reached = y; return true; }
    }
    uint8_t p = fabsf(err) > 12 ? 0 : 1;
    float ms = rate[p] > 0 ? DEAD_MS + 0.8f * fabsf(err) / rate[p] : startMs[p];
    uint16_t used;
    TRY(pulse(0, 0, err > 0 ? 100 : -100, p ? ROT_FINE_PWM : ROT_PWM, ms, &used));
    TRY(measure(us));
    float a;
    if (!io.yaw(&a) || io.lost() != lost0) return fail(RL_F("gyro_lost_data_in_rotation"));
    io.event(RL_F("rotate"), err > 0 ? int16_t(used) : -int16_t(used), p ? ROT_FINE_PWM : ROT_PWM, i16(y * 10), i16(a * 10));
    float turned = fabsf(a - y);
    if (turned < 1.0f) {
      rate[p] = 0;
      startMs[p] = used + 40 > T_MAX ? T_MAX : used + 40;   // static friction: push longer
    } else {
      float r = turned / (used - DEAD_MS > 10 ? used - DEAD_MS : 10);
      rate[p] = rate[p] > 0 ? 0.5f * rate[p] + 0.5f * r : r;
    }
  }
  if (!strict) {
    float y;
    if (reached && io.yaw(&y)) *reached = y;
    return true;
  }
  return fail(RL_F("rotation_not_reached"));
}

// The gentlest rotation that measurably turns the robot (>= 1.5 deg), lengthening by 30 ms.
bool Runner::rotatePulse(int8_t dir, float y0, int16_t us[NUS], float* y1, bool* turned) {
  *turned = false;
  for (uint16_t ms = T_MIN; ms <= 240; ms += 30) {
    TRY(pulse(0, 0, dir * 100, ROT_FINE_PWM, ms));
    TRY(measure(us));
    if (!io.yaw(y1)) return true;
    if (fabsf(*y1 - y0) >= 1.5f) { *turned = true; return true; }
  }
  return true;
}

// Heading from a capture taken square (N, turned checkpoints, boxes): a small gyro-measured
// probe gives each 45 deg beam's mm per degree; rotate by (target - reading) / slope. Beams
// outside 4-25 mm/deg are flipping across an edge or barely respond and are ignored; two beams
// disagreeing by > 8 deg, or none usable, keep the heading (and undo the probe).
bool Runner::squareToBeams(Pose pose, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  bool use[2] = {ref[L45] != NONE && ref[L45] >= 200, ref[R45] != NONE && ref[R45] >= 200};
  if (!use[0] && !use[1]) return true;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    float goal0 = goal, y0, y1;
    bool hadGoal = goalSet;
    if (!io.yaw(&y0)) return true;
    int16_t us1[NUS];
    bool turned;
    TRY(rotatePulse(+1, y0, us1, &y1, &turned));
    float fixes[2], slopes[2] = {0, 0};
    uint8_t nfix = 0;
    if (turned) {
      for (uint8_t b = 0; b < 2; ++b) {
        if (!use[b] || us[b] <= 0 || us1[b] <= 0 || fabsf(float(us1[b] - us[b])) < 10) continue;
        float s = (us1[b] - us[b]) / (y1 - y0);
        if (fabsf(s) < 4 || fabsf(s) > 25) continue;
        slopes[b] = s;
        fixes[nfix++] = (ref[b] - us1[b]) / s;
      }
    }
    bool disagree = nfix == 2 && fabsf(fixes[0] - fixes[1]) > 8;
    if (nfix == 0 || disagree) {
      io.event(disagree ? RL_F("beams_disagree") : RL_F("beams_unusable"), i16(slopes[0] * 10), i16(slopes[1] * 10));
      if (!hadGoal) { memcpy(us, us1, sizeof(int16_t) * NUS); return true; }
      setGoal(goal0);   // an unused probe turn must be undone
      TRY(rotateTo(goal0, 2, false));
      return measure(us);
    }
    float fix = clampf(nfix == 2 ? (fixes[0] + fixes[1]) / 2 : fixes[0], -12, 12);
    io.event(RL_F("beams_fix"), pose, i16(slopes[0] * 10), i16(slopes[1] * 10), i16(fix * 10));
    setGoal(y1 + fix);
    if (fabsf(fix) <= 1.5f) { memcpy(us, us1, sizeof(int16_t) * NUS); return true; }
    TRY(rotateTo(goal, 1.5f, false));
    TRY(measure(us));
  }
  return true;
}

// Steps 02/03 checkpoints: forward from the front sensor, sideways from the side walls; heading
// is held on the gyro and fixed separately (squareToBeams).
bool Runner::seek(Pose pose, bool creep, float tol, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  bool sideRef = ref[LSIDE] != NONE || ref[RSIDE] != NONE;
  TRY(measure(us));
  uint8_t lost = 0, jumps = 0, lostSide = 0, lastAxis = 1;
  int16_t prevFront = -1, movedX = 0;
  int16_t us2[NUS];
  for (uint8_t move = 0; move < 40; ++move) {
    float ex, ey, ex2, ey2;
    bool hasEx, hasEy, hasEx2, hasEy2;
    errors(pose, us, creep, &ex, &hasEx, &ey, &hasEy);
    io.event(RL_F("seek"), pose * 100 + move, hasEx ? i16(ex) : NONE, hasEy ? i16(ey) : NONE, us[FRONT]);
    bool jumped = hasEx && prevFront > 0 && fabsf(float(us[FRONT] - prevFront)) > 150 + movedX;
    if (jumped && hasEy && fabsf(ey) > tol) {   // beam slid off an edge after a sideways move
      io.event(RL_F("front_jump_sideways_first"), prevFront, us[FRONT]);
      TRY(translate(1, ey, us));
      TRY(hold(us));
      continue;
    }
    if (jumped && ++jumps <= 2) { TRY(measure(us)); continue; }
    jumps = 0;
    if (!hasEx) {   // keep movedX: the next good reading may still follow a forward move
      if (++lost > 2) return fail(RL_F("front_reading_lost"));
      TRY(measure(us));
      continue;
    }
    lost = 0;
    prevFront = us[FRONT];
    movedX = 0;
    bool xok = fabsf(ex) <= tol;
    bool yok = (!hasEy && !sideRef && !creep) || (hasEy && fabsf(ey) <= tol);
    if (xok && yok) { io.event(RL_F("reached"), pose, move); return true; }
    bool bothOff = !xok && hasEy && !yok;
    if (bothOff && fabsf(ex) < 400) lastAxis = 0;   // close in: line up sideways first
    uint16_t used;
    if (!xok && (yok || !hasEy || (bothOff && lastAxis == 1))) {
      lastAxis = 0;
      TRY(translate(0, ex, us2, T_MAX, &used));
      movedX = 1000;
      errors(pose, us2, creep, &ex2, &hasEx2, &ey2, &hasEy2);
      TRY(learn(0, true, ex, hasEx2, ex2, used));
      TRY(hold(us2));
    } else if (hasEy && !yok) {
      lastAxis = 1;
      TRY(translate(1, ey, us2, T_MAX, &used));
      errors(pose, us2, creep, &ex2, &hasEx2, &ey2, &hasEy2);
      TRY(learn(1, true, ey, hasEy2, ey2, used));
      TRY(hold(us2));
    } else if (!xok) {
      TRY(translate(0, ex, us2, T_MAX, &used));
      movedX = 1000;
      errors(pose, us2, creep, &ex2, &hasEx2, &ey2, &hasEy2);
      TRY(learn(0, true, ex, hasEx2, ex2, used));
      TRY(hold(us2));
    } else if (sideRef) {
      if (lostSide++ >= 2) return fail(RL_F("side_walls_not_where_the_capture_saw_them"));
      TRY(measure(us));
      continue;
    } else {
      io.event(RL_F("reached_no_sideways"), pose, move);
      return true;
    }
    memcpy(us, us2, sizeof(us2));
  }
  return fail(RL_F("checkpoint_not_reached"));
}

// Match a checkpoint on every sensor it has a target for (N: all five; P: front and the 45s):
// mismatch = sum of |reading - target|, each capped at 300 mm (a missing echo counts 300), so one
// sensor seeing something else cannot dominate. Forward/back directly from the front sensor;
// sideways and heading by trial moves (strafe left/right, turn on the gyro) kept only if the
// mismatch drops by > 8 mm; steps shrink when nothing helps. Done at ~25 mm a sensor; otherwise
// stop rather than go on from the wrong place.
bool Runner::descend(Pose pose, bool turns, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  bool key[NUS];
  uint8_t nkeys = 0;
  for (uint8_t k = 0; k < NUS; ++k) { key[k] = ref[k] != NONE; nkeys += key[k]; }
  if (!key[FRONT]) return fail(RL_F("checkpoint_has_no_front_target"));
  float e[NUS], e2[NUS], cost, cost2;
  bool h[NUS], h2[NUS];
  int16_t u2[NUS];
  #define LOOK(U, E, H, C) do {                                                      \
      for (uint8_t t_ = 0; t_ < 3; ++t_) {                                           \
        TRY(measure(U));                                                             \
        bool all_ = true;                                                            \
        for (uint8_t k_ = 0; k_ < NUS; ++k_) if (key[k_] && U[k_] <= 0) all_ = false;  \
        if (all_) break;                                                             \
      }                                                                              \
      C = 0;                                                                         \
      for (uint8_t k_ = 0; k_ < NUS; ++k_) {                                         \
        H[k_] = U[k_] > 0;                                                           \
        E[k_] = float(U[k_] - ref[k_]);                                              \
        if (key[k_]) C += H[k_] ? (fabsf(E[k_]) < 300 ? fabsf(E[k_]) : 300) : 300;    \
      }                                                                              \
    } while (0)
  float yMs = 300, wDeg = 8, lastTurn = 8;
  LOOK(us, e, h, cost);
  io.event(RL_F("match_start"), pose, i16(cost), turns);
  int8_t keep = -1;   // the trial that last helped: try it first again
  for (uint8_t move = 0; move < 40; ++move) {
    if (cost <= 25.0f * nkeys) {
      io.event(RL_F("match_reached"), pose, i16(cost), i16(e[FRONT]));
      return true;
    }
    // Both side walls seen with the capture's spacing: the sideways offset is measured, not
    // guessed (simulation: 28 cm off at N, both sides "useless" to the trials, so it only turned).
    if (key[LSIDE] && key[RSIDE] && h[LSIDE] && h[RSIDE] &&
        fabsf(float((us[LSIDE] + us[RSIDE]) - (ref[LSIDE] + ref[RSIDE]))) <= 100) {
      float ey = ((us[LSIDE] - us[RSIDE]) - (ref[LSIDE] - ref[RSIDE])) / 2.0f;
      if (fabsf(ey) > 30) {
        TRY(translate(1, ey, us));
        TRY(hold(us));
        LOOK(us, e, h, cost);
        io.event(RL_F("match_sideways"), pose, i16(cost), i16(ey));
        continue;
      }
    }
    // Sideways signal: the side walls when the checkpoint has both and both are seen, else the
    // 45 deg pair. Either way a strafe right lengthens the left reading and shortens the right.
    bool sides = key[LSIDE] && key[RSIDE] && h[LSIDE] && h[RSIDE];
    uint8_t l = sides ? LSIDE : L45, r = sides ? RSIDE : R45;
    bool pair = key[l] && key[r] && h[l] && h[r];
    bool pairBad = !pair || fabsf(e[l]) >= 250 || fabsf(e[r]) >= 250;
    // While the pair is useless the robot is most likely turned away: turns first, forward only
    // if far off. Never a long reverse on a short front reading (it may be another surface).
    if (h[FRONT] && fabsf(e[FRONT]) > (pairBad && turns ? 150 : 30) && e[FRONT] > -250) {
      TRY(translate(0, e[FRONT], us));
      TRY(hold(us));
      LOOK(us, e, h, cost);
      io.event(RL_F("match_forward"), pose, i16(cost), i16(e[FRONT]));
      continue;
    }
    // Trials: 0 strafe left, 1 strafe right, 2 turn CCW, 3 turn CW.
    bool beams = key[L45] && key[R45] && h[L45] && h[R45];
    bool twisted = beams && (e[L45] > 0) == (e[R45] > 0) &&
                   (fabsf(e[L45]) < fabsf(e[R45]) ? fabsf(e[L45]) : fabsf(e[R45])) > 20;
    uint8_t order[4], n = 4;
    if (!turns) {
      n = 2;
      order[0] = pair && e[l] - e[r] < 0 ? 1 : 0;
      order[1] = 1 - order[0];
    } else if (pairBad || twisted) {
      order[0] = 2; order[1] = 3; order[2] = 0; order[3] = 1;
    } else if (e[l] - e[r] < 0) {
      order[0] = 1; order[1] = 0; order[2] = 2; order[3] = 3;
    } else {
      order[0] = 0; order[1] = 1; order[2] = 2; order[3] = 3;
    }
    for (uint8_t i = 1; i < n && keep >= 0; ++i) {
      if (order[i] == keep) { order[i] = order[0]; order[0] = uint8_t(keep); break; }
    }
    bool improved = false;
    for (uint8_t i = 0; i < n && !improved; ++i) {
      uint8_t t = order[i];
      for (uint8_t pass = 0; pass < 2; ++pass) {   // pass 0 the trial, pass 1 its undo
        bool left = (t == 0) != (pass == 1), ccw = (t == 2) != (pass == 1);
        if (t < 2) {
          int8_t d = left ? 1 : -1;
          TRY(pulse(0, d * 100, wcomp[1][d > 0], PWM_Y, yMs));
          TRY(measure(u2));
          TRY(hold(u2));
        } else {
          float deg = pass == 1 ? lastTurn : (pairBad ? 15 : wDeg);   // coarse while far off
          lastTurn = deg;
          goal += ccw ? deg : -deg;
          TRY(rotateTo(goal, 2, false));
        }
        if (pass == 1) break;
        LOOK(u2, e2, h2, cost2);
        bool ok = cost2 < cost - 8;
        io.event(RL_F("match_try"), t, i16(cost), i16(cost2), ok);
        if (ok) {
          memcpy(us, u2, sizeof(u2));
          memcpy(e, e2, sizeof(e));
          memcpy(h, h2, sizeof(h));
          cost = cost2;
          keep = int8_t(t);
          improved = true;
          break;
        }
      }
    }
    if (!improved) {
      keep = -1;
      if (yMs <= T_MIN && wDeg <= 4) break;
      yMs = yMs * 0.7f < T_MIN ? T_MIN : yMs * 0.7f;
      wDeg = wDeg * 0.7f < 4 ? 4 : wDeg * 0.7f;
      LOOK(us, e, h, cost);
    }
  }
  #undef LOOK
  return fail(RL_F("checkpoint_match_not_reached"));
}

// Start: drive forward about `mm` on the held start heading (the robot is placed facing the main
// wall), in short moves, never closer than 150 mm short of N's front reading.
bool Runner::approach(int16_t mm, int16_t us[NUS]) {
  for (uint8_t t = 0; t < 3; ++t) { TRY(measure(us)); if (us[FRONT] > 0) break; }
  if (us[FRONT] <= 0) return fail(RL_F("approach_front_sees_nothing"));
  int16_t target = us[FRONT] - mm, nearest = cal.ref[N][FRONT] + 150;
  if (target < nearest) target = nearest;
  io.event(RL_F("approach"), us[FRONT], target);
  for (uint8_t move = 0; move < 8; ++move) {
    if (us[FRONT] > 0 && us[FRONT] - target <= 25) return true;
    if (us[FRONT] <= 0) { TRY(measure(us)); continue; }
    TRY(translate(0, us[FRONT] - target, us));
    TRY(hold(us));
  }
  return true;   // best effort: the N match takes over from wherever it got
}

// Square to the main wall where the robot is (it faces the wall within ~30 deg): sweep +/-40 deg;
// square = middle of the contiguous front readings within 30 mm of the least near the middle.
// The wide beam cone makes a match on readings alone able to hide a twist.
bool Runner::squareHere() {
  int16_t us[NUS];
  TRY(measure(us));
  float y0, y;
  if (!io.yaw(&y0)) return fail(RL_F("gyro_not_valid"));
  TRY(rotateTo(y0 - 40, 4, true));
  float yaws[20];
  int16_t fronts[20];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 20; ++i) {
    TRY(measure(us));
    if (!io.yaw(&y)) return fail(RL_F("gyro_lost_data_in_sweep"));
    if (us[FRONT] > 0) { yaws[n] = y; fronts[n] = us[FRONT]; ++n; }
    io.event(RL_F("sweep"), i16((y - y0) * 10), us[FRONT]);
    if (y >= y0 + 40) break;
    float y1;
    bool turned;
    TRY(rotatePulse(+1, y, us, &y1, &turned));
  }
  if (n < 4) return fail(RL_F("sweep_saw_no_wall"));
  // Flat stretches: neighbouring front readings within 30 mm. The main wall, seen square, gives
  // the widest one (the beam cone keeps it flat over ~25 deg); a side wall's end gives a short,
  // sloping one that can read closer (simulation: "least reading" picked one, 30 deg off).
  // Take the widest stretch whose middle is within 35 deg of where the robot started the sweep.
  int8_t bestLo = -1, bestHi = -1;
  for (uint8_t a = 0; a < n;) {
    uint8_t b = a;
    while (b + 1 < n && fabsf(float(fronts[b + 1] - fronts[b])) <= 30) ++b;
    float mid = (yaws[a] + yaws[b]) / 2 - y0;
    bool better = bestLo < 0 || b - a > bestHi - bestLo ||
                  (b - a == bestHi - bestLo && fabsf(mid) < fabsf((yaws[bestLo] + yaws[bestHi]) / 2 - y0));
    if (fabsf(mid) <= 35 && b > a && better) { bestLo = int8_t(a); bestHi = int8_t(b); }
    a = b + 1;
  }
  if (bestLo < 0) return fail(RL_F("sweep_saw_no_wall_ahead"));
  uint8_t lo = uint8_t(bestLo), hi = uint8_t(bestHi);
  int16_t least = fronts[lo];
  for (uint8_t j = lo; j <= hi; ++j) if (fronts[j] < least) least = fronts[j];
  float normal = (yaws[lo] + yaws[hi]) / 2;
  io.event(RL_F("square"), i16((normal - y0) * 10), least, i16((yaws[lo] - y0) * 10), i16((yaws[hi] - y0) * 10));
  setGoal(normal);
  return rotateTo(normal, 2, false);
}

// The colour turn: 90 deg from the held heading (or the current one) on the gyro.
bool Runner::turnTo(bool ccw) {
  int16_t us[NUS];
  TRY(measure(us));
  float start, y;
  if (goalSet && io.lost() == goalLost) start = goal;
  else if (!io.yaw(&start)) return fail(RL_F("gyro_not_valid_for_the_turn"));
  setGoal(start + (ccw ? 90 : -90));
  TRY(rotateTo(goal, HOLD_TOL, true, &y));
  io.event(RL_F("turned"), ccw, i16((y - start) * 10));
  return true;
}

// Final approach: creep straight forward to the box's front reading (half the remaining
// distance per move, none once within 15 mm), strafe only if the back-wall side drifts > 30 mm,
// hold the heading, then square on the box's 45 deg beam.
bool Runner::park(Pose pose, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  int8_t side = ref[RSIDE] != NONE ? RSIDE : ref[LSIDE] != NONE ? LSIDE : -1;
  TRY(measure(us));
  int16_t prev = -1;
  bool squared = false;
  for (uint8_t move = 0; move < 40; ++move) {
    int16_t front = us[FRONT];
    bool hasEy = false;
    float ey = 0;
    if (side >= 0 && us[side] > 0 && fabsf(float(us[side] - ref[side])) < SIDE_WINDOW) {
      ey = side == RSIDE ? float(ref[side] - us[side]) : float(us[side] - ref[side]);
      hasEy = true;
    }
    io.event(RL_F("park"), move, front, hasEy ? i16(ey) : NONE);
    if (front <= 0 || (prev > 0 && fabsf(float(front - prev)) > 150)) {   // lost or implausible
      prev = -1;
      TRY(measure(us));
      continue;
    }
    prev = front;
    if (hasEy && fabsf(ey) > 30) {
      TRY(translate(1, ey, us));
    } else if (front > ref[FRONT] + 15) {   // the shortest forward move still travels ~20 mm
      TRY(translate(0, (front - ref[FRONT]) / FRACTION * 0.5f, us, 200));
    } else {
      float y;
      bool hy = io.yaw(&y);
      if (hy && goalSet && fabsf(goal - y) > HOLD_TOL) { TRY(hold(us)); continue; }
      if (!squared) {
        squared = true;
        TRY(squareToBeams(pose, us));
        continue;
      }
      io.event(RL_F("parked"), front, ref[FRONT], hy && goalSet ? i16((y - goal) * 10) : NONE);
      return true;
    }
    TRY(hold(us));
  }
  return fail(RL_F("park_not_finished"));
}

bool Runner::run(uint8_t first, uint8_t last, int8_t forceTurn) {
  why = 0;
  int16_t us[NUS];
  float y, rg = 0;
  TRY(measure(us));
  if (!io.yaw(&y)) return fail(RL_F("gyro_not_zeroed"));
  setGoal(y);   // placed pointing at the wall: hold this heading until P squares it
  if (first <= 1 && last >= 1) {
    // P is skipped (only the front sees the main wall from there, so its sideways position and
    // heading were never reliable): forward ~30 cm, then square to the wall from closer in.
    io.event(RL_F("step"), 1);
    TRY(approach(300, us));
    TRY(squareHere());
  }
  if (first <= 2 && last >= 2) {
    // N: all five sensors see real surfaces (main wall, both side walls), so match them all.
    io.event(RL_F("step"), 2);
    TRY(descend(N, true, us));
    TRY(squareHere());
    TRY(descend(N, false, us));
    TRY(seek(N, false, 20, us));
  }
  turnDir = forceTurn;
  if (first <= 3 && last >= 3) {
    io.event(RL_F("step"), 3);
    TRY(measure(us, &rg));
    float g = cal.greenMilli / 1000.0f, r = cal.redMilli / 1000.0f;
    if (rg <= 0 || g <= 0 || r <= g) return fail(RL_F("no_colour_reading"));
    float split = sqrtf(g * r);
    float conf = clampf(fabsf(logf(rg / split)) / (logf(r / g) / 2), 0, 1);
    turnDir = rg > split ? -1 : 1;   // red turns right, green turns left
    io.event(RL_F("colour"), i16(rg * 1000), i16(conf * 100), turnDir);
    if (conf < 0.5f) return fail(RL_F("colour_unclear"));
    TRY(turnTo(turnDir > 0));
    Pose turned = turnDir > 0 ? N_LEFT : N_RIGHT;
    TRY(seek(turned, false, 40, us));
    TRY(squareToBeams(turned, us));   // square before the drive into the box
    TRY(seek(turned, false, 40, us));
  }
  if (first <= 4 && last >= 4) {
    io.event(RL_F("step"), 4);
    if (turnDir == 0) return fail(RL_F("no_turn_direction"));
    TRY(park(turnDir > 0 ? BOX_LEFT : BOX_RIGHT, us));
  }
  io.event(RL_F("done"));
  return true;
}

}  // namespace rl

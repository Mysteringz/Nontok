// Port of tools/seek_steps.py (the version run on the field on 30 Oct). Comments there explain
// each rule and the field observation behind it; the numbers here must stay in step with it.
#include "run_logic.h"
#include <math.h>
#include <string.h>

namespace rl {

#define TRY(x) do { if (!(x)) return false; } while (0)

// Constants the host simulator's tuner may change (tools/tune_run_logic.py); fixed on the Mega.
#ifdef __AVR__
#define TUNE(type) static const type
#else
#define TUNE(type) type
#endif
TUNE(uint8_t) PWM_X = 85; TUNE(uint8_t) PWM_Y = 80;   // timed strafe; 2 Nov field: 100 was twice too fast
static const float HOLD_TOL = 2.0f;     // deg: brake-ended pulses sized by the move model turn ~1 deg at the least
static const float ROT_TOL = 1.5f;      // deg: where a correction aims to finish (2 Nov field: 0.8 kept overshooting)
static const float ROT_COARSE_DEG = 15; // more than this: one long smooth pulse at the coarse power
static const float FRACTION = 0.85f;    // cover this share of the error per timed translation
static const uint16_t T_MIN = 50, T_MAX = 300;    // timed translations: long moves veer
static const uint16_t ROT_T_MAX = 2100; // ms: + ramp-down and brake stays inside the gyro FIFO (yaw only: 2.5 s)
static const float DRIVE_MAX_MS = 2000; // ms: short correcting drives stay inside the gyro FIFO
static const float CREEP_MAX_MS = 20000; // ms: one continuous creep to N or into the box (the gyro FIFO may
                                         // overflow: the heading is then re-based, not corrected, on the way)
TUNE(uint8_t) CREEP_FLOOR = 41;   // PWM: creep speed at the target, raised if the robot does not move
static const int16_t SIDE_WINDOW = 250; // a side reading this far off its target is another surface
TUNE(float) STEEP_ON = 0;            // 0 (user 3 Nov evening: back to Reinforced + adjust): no steep check / search, no front restart, no near-N square-up
TUNE(uint8_t) DRIVE_PWM = 45;    // creep top speed; 2 Nov field: 52 twice too fast, 47 a tiny bit too fast
TUNE(uint8_t) SIDE_PWM = 68;
TUNE(uint8_t) ROT_PWM_C = 62;    // coarse turns (2 Nov field: 1/5 speed)
TUNE(uint8_t) ROT_PWM_F = 56;    // fine turns
TUNE(float) CREEP_SLOW = 250;     // mm out where the creep starts slowing to CREEP_FLOOR
// Slower near the main wall and in the box (user, 3 Nov: the ultrasonics are sometimes not fast enough, a
// ping every ~25-50 ms): top speed, end speed and where the slowing starts, for steps 1-2 and for parking.
// The floor still rises by 4 if the robot does not move (breakaway differs with the battery).
TUNE(uint8_t) WALL_PWM = 45;      // creep top speed toward the main wall / N (was DRIVE_PWM 45)
TUNE(uint8_t) WALL_WALK_PWM = 255; // the steep / angled start walk across the open floor (was START_PWM 52-55)
TUNE(uint8_t) WALL_FLOOR = 41;    // (as CREEP_FLOOR; 40 stalled more often in the sim, then the gyro FIFO ran over)
TUNE(float) WALL_SLOW = 250;      // mm (was 250)
TUNE(uint8_t) PARK_PWM = 45;      // creep top speed in step 4: to the box, into it, the final fixes
TUNE(uint8_t) PARK_FLOOR = 41;
TUNE(float) PARK_SLOW = 250;      // mm
TUNE(uint8_t) PARK_SIDE_PWM = 68; // strafes in step 4 (was SIDE_PWM 68; raised by 4 if it does not move)
// 3 Nov sim, these against the previous speeds: parked 99.3 / 99.0 / 98.5 % (+-45, random fields, +-60) vs
// 98.9 / 98.4 / 98.4 %, no more wall touches, ~8 s longer; slower still (39-40 floors) lost heading to the gyro FIFO     // continuous strafes; 2 Nov field: 80 was twice too fast
// Sensor sets: N uses all five; after a right turn (and its box) front, front-left (L45) and left side;
// after a left turn front, front-right (R45) and right side.
enum { SET_ALL = 0, SET_RIGHT_TURN = 1, SET_LEFT_TURN = 2 };
static const int16_t BLOCK_MM = 20;
#ifdef TEAM_PARK   // the team's parking stage (373ff50) unchanged, for comparison
static const bool PARK_ORIG = true;
#else
static const bool PARK_ORIG = false;
#endif
TUNE(float) CORNER_MARGIN = 25;   // mm: a 45 deg echo must be this much shorter than the front's to be a corner
TUNE(float) CORNER_ZONE = 450;  // mm of front reading: the 10 cm corner stop applies only closer in     // 2 cm rule: no move toward a sensor reading this or less

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static int16_t i16(float v) { return int16_t(clampf(v, -32000, 32000)); }

Runner::Runner(Io& io_, const Calib& cal_)
    : io(io_), cal(cal_), why(0), goal(0), goalSet(false), goalLost(0), turnDir(0), snapSum(0), snapped(false) {
  back.init(0.35f, 70);    // mm/ms beyond the soft-start dead time
  side.init(0.25f, 80);
  // Field 30 Oct: 120 / 80 PWM turned the robot; 80 PWM turned ~7 deg in a 120 ms coasting pulse.
  // 2 Nov field: 110 / 80 PWM turned far too fast; ~1/5 of that speed: just above breakaway (raised by
  // rotateTo if the robot does not turn at all)
  // 2 Nov run: 62 / 56 PWM turned ~0.1 deg/ms (100 deg/s) and the first turns overshot: lower power,
  // rates from that run as the starting point (still raised if the robot does not turn)
  // 62 / 56 PWM (2 Nov field: 54 / 50 under-turned). Starting rates measured on the 2 Nov run at these
  // powers (0.096 / ~0.105 deg/ms): with lower guesses the first long turn ran ~2x too far.
  rotPwm[0] = ROT_PWM_C; rot[0].init(0.096f * (ROT_PWM_C - 40) / 22.0f, 75);   // deg/ms (rate ~ power above breakaway)
  rotPwm[1] = ROT_PWM_F; rot[1].init(0.105f * (ROT_PWM_F - 40) / 16.0f, 70);
  rotMinMs[0] = rotMinMs[1] = 0;
  sweepRate = 0.08f;
  memset(wcomp, 0, sizeof(wcomp));
  stopTau = 0.15f;
  beamSlope[0] = beamSlope[1] = beamSlope[2] = 0;
  blocked = 0;
  tpd = 0;
  coast = 0;
  cornered = false;
  inPark = false;
  wallPhase = false;
  skewMode = 0;
  stopK = FRONT;
  memset(pre, -1, sizeof(pre));
  rN = 0;
  stopTauY = 0.08f;
  cornerMm = 100;  // mm: forward drives stop if a 45 deg beam reads less (corner closing on a wall)
  fixRate0[0] = 0.15f; fixRate0[1] = 0.08f; fixRate0[2] = 0.03f;   // mm/ms, mm/ms, deg/ms at FIX_PWM (learned)
  for (uint8_t i = 0; i < 3; ++i) fixRate[i] = fixRate0[i];   // cautious until learned: 2 Nov the robot rolled ~20 cm after a stop from 380 mm/s
}

bool Runner::measure(int16_t us[NUS], float* rg) {
  float dummy;
  if (!io.measure(us, rg ? rg : &dummy)) return fail(RL_F("stopped"));
  // One stray echo among the 3 scans voids a sensor's reading (~7 % of measurements). The front steers
  // every stage (3 Nov sim: a void front skipped the creep into the box, 185 mm short): scan once more and
  // fill in whatever came back void.
  if (us[FRONT] <= 0) {
    int16_t again[NUS];
    if (!io.measure(again, &dummy)) return fail(RL_F("stopped"));
    for (uint8_t k = 0; k < NUS; ++k) if (us[k] <= 0) us[k] = again[k];
  }
  noteWalls(us);
  return true;
}

void Runner::noteWalls(const int16_t* z) {
  for (uint8_t k = 0; k < NUS; ++k) {
    if (z[k] <= 0) continue;   // no echo or not pinged: keep what is known
    if (z[k] <= BLOCK_MM) blocked |= 1 << k;
    else if (z[k] > 2 * BLOCK_MM) blocked &= ~(1 << k);
  }
}

bool Runner::towardWall(int8_t x, int8_t y) const {
  return (x > 0 && (blocked & ((1 << FRONT) | (1 << L45) | (1 << R45)))) ||
         (y > 0 && (blocked & ((1 << LSIDE) | (1 << L45)))) || (y < 0 && (blocked & ((1 << RSIDE) | (1 << R45))));
}

bool Runner::pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, float ms, uint16_t* used, uint16_t lo, uint16_t hi) {
  uint16_t m = uint16_t(clampf(ms, lo, hi));
  if (used) *used = m;
  if (towardWall(x, y)) { io.event(RL_F("wall_block"), x, y, blocked); if (used) *used = 0; return true; }
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
  if (axis == 0 && us[FRONT] > 0) {
    // Forward or back: continuous Kalman-tracked drive on the front sensor (see driveForward), never
    // a fixed-length jerk, the same in both directions (2 Nov: a timed reverse overshot and the robot
    // went in and out of N). The stop target is 90 % of the way; backing off at most 200 mm at a time.
    float stopAt = us[FRONT] - 0.9f * err;
    if (stopAt < 20) stopAt = 20;
    if (stopAt > us[FRONT] + 200) stopAt = us[FRONT] + 200;
    if (used) *used = 0;
    return driveForward(stopAt, DRIVE_MAX_MS, DRIVE_PWM, us);
  }
  // Timed move sized by this run's move model (reverse or strafe): a small error, a small move.
  MoveModel& m = axis == 1 ? side : back;
  float ms = m.msFor(FRACTION * fabsf(err));
  if (ms > maxMs) ms = maxMs;
  if (axis == 1 && ms > 220) ms = 220;   // a 300 ms strafe also rotated the robot
  int8_t sign = err > 0 ? 1 : -1;
  float before, after;
  bool hasBefore = io.yaw(&before);
  int8_t w = wcomp[axis][sign > 0];
  int16_t was[NUS];
  memcpy(was, us, sizeof(was));
  uint16_t ran;
  TRY(pulse(axis == 0 ? sign * 100 : 0, axis == 1 ? sign * 100 : 0, w, axis == 0 ? PWM_X : PWM_Y, ms, &ran, T_MIN, T_MAX));
  if (used) *used = ran;
  TRY(measure(us));
  // Learn the move size from the sensors that measure it directly: the front for a reverse, the side
  // walls for a strafe (left: the left reading shrinks, the right one grows).
  float moved = 0;
  uint8_t n = 0;
  if (axis == 0) {
    if (was[FRONT] > 0 && us[FRONT] > 0 && fabsf(float(us[FRONT] - was[FRONT])) < 400) { moved = float(us[FRONT] - was[FRONT]); n = 1; }
  } else {
    if (was[LSIDE] > 0 && us[LSIDE] > 0 && fabsf(float(us[LSIDE] - was[LSIDE])) < 300) { moved += sign * float(was[LSIDE] - us[LSIDE]); ++n; }
    if (was[RSIDE] > 0 && us[RSIDE] > 0 && fabsf(float(us[RSIDE] - was[RSIDE])) < 300) { moved += sign * float(us[RSIDE] - was[RSIDE]); ++n; }
  }
  if (n) m.learn(ran, clampf(moved / n, 0, 1000));
  if (hasBefore && io.yaw(&after)) {
    float gain = axis == 1 ? 2.0f : 1.5f;
    wcomp[axis][sign > 0] = int8_t(clampf(roundf(w - gain * (after - before)), -40, 40));
  }
  return true;
}

// Continuous drive toward a front-distance target at one constant slow speed, with no fixed move
// length: forward when the front reads more than `stopAt`, backward when less (s = +1 / -1). All five ultrasonics are pinged in turn while moving (front ~20 Hz, the others ~5 Hz,
// see Io::drivePoll) and each is tracked by its own Kalman filter (distance, closing speed; echoes
// far from the prediction are rejected). The robot stops once the predicted wall distance after the
// stop (front distance + speed * stopTau, tau learned from where each drive really stopped) reaches
// `stopAt`, so a move cannot overshoot. It stops for good after maxMs (<= DRIVE_MAX_MS: the gyro
// FIFO, yaw only, holds 2.5 s), after ~200 ms without a front echo (4 front pings), or when the robot does not move (the
// speed is then raised in steps of 8 PWM up to 100). The side and 45 deg filters are measured and
// logged ("dside"); they do not steer.
bool Runner::driveForward(float stopAt, float maxMs, uint8_t pwm, int16_t us[NUS]) {
  const uint8_t sk = stopK;
  if (us[sk] <= 0) return fail(RL_F("drive_needs_front_echo"));
  if (maxMs > CREEP_MAX_MS) maxMs = CREEP_MAX_MS;
  if (inPark && pwm > PARK_PWM) pwm = PARK_PWM;
  else if (wallPhase && !inPark) { uint8_t cap = pwm > DRIVE_PWM ? WALL_WALK_PWM : WALL_PWM; if (pwm > cap) pwm = cap; }
  const uint8_t pwmMax = pwm;
  const uint8_t fl = inPark ? PARK_FLOOR : wallPhase ? WALL_FLOOR : CREEP_FLOOR;
  const float slowMm = inPark ? PARK_SLOW : wallPhase ? WALL_SLOW : CREEP_SLOW;
  uint8_t floorPwm = fl < pwmMax ? fl : pwmMax;
  Kalman1D kf[NUS];
  bool live[NUS];      // filter started: from the stopped reading, or the first echo while driving
  uint8_t blind[NUS];  // pings in a row without an accepted echo
  uint8_t okc[NUS];    // echoes accepted since the filter started: the heading fit trusts it from 2 on
  for (uint8_t k = 0; k < NUS; ++k) {
    live[k] = us[k] > 0;
    if (live[k]) kf[k].init(float(us[k]));
    blind[k] = 0;
    okc[k] = 0;
  }
  float d0 = float(us[sk]), t = 0;
  const int8_t s = d0 > stopAt ? 1 : -1;   // +1 forward (front shrinks), -1 backward
  int8_t w = wcomp[0][s > 0];
  float before = 0, after;
  bool hasBefore = io.yaw(&before);
  cornered = false;
  if (towardWall(s * 100, 0)) { io.event(RL_F("wall_block"), s * 100, 0, blocked); cornered = true; return true; }
  // a heading-fit drive pings the side walls it fits ~3x as often (main.cpp schedules): 3 Nov sim, at 4 Hz the
  // side fit over the last 19 cm into the box was off by up to 3 deg and turned the robot the wrong way
  const uint8_t focus = sk == L45 ? FOCUS_LEFT : sk == R45 ? FOCUS_RIGHT : PARK_ORIG && inPark ? FOCUS_FRONT : skewMode == 1 + SET_ALL ? FOCUS_ALL : skewMode == 1 + SET_RIGHT_TURN ? FOCUS_LEFT
                      : skewMode == 1 + SET_LEFT_TURN ? FOCUS_RIGHT : FOCUS_FRONT;
  if (!io.driveStart(s * 100, 0, w, pwm, focus)) return fail(RL_F("stopped"));
  uint16_t polls = 0;
  float tWin = 0, dWin = d0;
  int16_t lastFront = us[FRONT];
  int16_t raw45[2] = {us[L45], us[R45]};   // last raw echoes: a corner stop needs one really close
  float dDecision = d0, vDecision = 0;
  snapped = false;
  rN = rF = rL = rFF = rFL = 0;
  rFmin = 1e6f; rFmax = -1e6f;
  uint8_t rejRun = 0;    // front echoes rejected in a row that agree with each other
  float rejLast = 0;
  for (;;) {
    int16_t z[NUS];
    float dt;
    if (!io.drivePoll(z, &dt)) { io.driveEnd(); return fail(RL_F("stopped")); }
    t += dt * 1000;
    noteWalls(z);
    if (towardWall(s * 100, 0)) { io.event(RL_F("wall_block"), s * 100, 0, blocked); cornered = true; break; }
    for (uint8_t k = 0; k < NUS; ++k) {
      if (live[k]) kf[k].predict(dt);
      if (z[k] == NOT_PINGED) continue;
      if (z[k] > 0) {
        // A side or 45 deg sensor rejected 3 pings running is seeing a different surface (a wall
        // ended, or the first echo was spurious): restart its filter there. The front never restarts.
        if (!live[k] || (k != FRONT && k != sk && blind[k] >= 3)) { kf[k].init(float(z[k])); live[k] = true; blind[k] = 0; okc[k] = 0; }
        else if (kf[k].update(float(z[k]))) { blind[k] = 0; if (okc[k] < 255) ++okc[k]; if (k == FRONT) rejRun = 0; }
        else {
          ++blind[k];   // rejected as spurious: counts like a missing echo
          // The front restarts only on a NEARER surface that 3 echoes in a row agree on (3 Nov field, IMG_4001:
          // the filter started on the far corner's 1726 mm at a steep start and threw away the main wall's
          // 1190, 895, 571 mm: the robot drove on past N almost into the wall). A farther one is never trusted.
          if (k == FRONT) {
            rejRun = rejRun && fabsf(z[k] - rejLast) < 80 ? rejRun + 1 : 1;
            rejLast = z[k];
            if (STEEP_ON > 0.5f && rejRun >= 3 && z[k] < kf[k].dist() - 120) {
              io.event(RL_F("front_restart"), i16(kf[k].dist()), z[k]);
              kf[k].init(float(z[k])); blind[k] = 0; rejRun = 0;
            }
          }
        }
      } else ++blind[k];
    }
    if (z[FRONT] != NOT_PINGED) lastFront = z[FRONT];
    if (z[L45] != NOT_PINGED) raw45[0] = z[L45];
    if (z[R45] != NOT_PINGED) raw45[1] = z[R45];
    float d = kf[sk].dist(), v = kf[sk].speed();
    // First moment both side walls are tracked at the capture's spacing: snapshot for skewFrom (the
    // heading over the rest of this straight drive)
    if (snapSum && !snapped && live[LSIDE] && live[RSIDE] && blind[LSIDE] == 0 && blind[RSIDE] == 0 &&
        fabsf(kf[LSIDE].dist() + kf[RSIDE].dist() - snapSum) < 120) {
      for (uint8_t k = 0; k < NUS; ++k) snap[k] = live[k] ? i16(kf[k].dist()) : -1;
      snapped = true;
    }
    // heading fit: each accepted front echo against the side offset of that moment (filtered sides)
    if (skewMode && z[FRONT] > 0 && blind[FRONT] == 0) {
      uint8_t set = skewMode - 1;
      // a side filter seeded by a stray reading (3 Nov sim: 1458 mm with the wall at 215) is only trusted once
      // two echoes have confirmed it: one such sample made the fit read +8 deg on a robot 2 deg off
      bool l = set != SET_LEFT_TURN && live[LSIDE] && blind[LSIDE] < 2 && okc[LSIDE] >= 2,
           r = set != SET_RIGHT_TURN && live[RSIDE] && blind[RSIDE] < 2 && okc[RSIDE] >= 2;
      if (set == SET_ALL) { if (!(l && r) || fabsf(kf[LSIDE].dist() + kf[RSIDE].dist() - (cal.ref[N][LSIDE] + cal.ref[N][RSIDE])) > 120) l = r = false; }
      if (l || r) {
        float lat = l && r ? (kf[LSIDE].dist() - kf[RSIDE].dist()) / 2 : l ? kf[LSIDE].dist() : -kf[RSIDE].dist();
        float f = kf[FRONT].dist();
        rN += 1; rF += f; rL += lat; rFF += f * f; rFL += f * lat;
        if (f < rFmin) rFmin = f;
        if (f > rFmax) rFmax = f;
      }
    }
    // Creep: speed falls with the distance left (full pwmMax 250 mm out, floorPwm at the target), so
    // the robot slows smoothly into the stop. Not moving at all over 400 ms raises only the floor, by 4.
    if (t - tWin >= 400) {
      if (s * (dWin - d) < 3 && floorPwm < 75) floorPwm += 4;
      tWin = t;
      dWin = d;
    }
    float rem = clampf(s * (d - stopAt) / slowMm, 0, 1);
    uint8_t want = uint8_t(floorPwm + (pwmMax > floorPwm ? (pwmMax - floorPwm) * rem : 0) + 0.5f);
    if (want != pwm && (want + 2 <= pwm || want >= pwm + 2 || want == floorPwm)) {
      pwm = want;
      if (!io.driveSet(s * 100, 0, w, pwm)) { io.driveEnd(); return fail(RL_F("stopped")); }
    }
    dDecision = d;
    vDecision = v;
    // Sparse trace, one line per 125 ms so the 9600-baud Bluetooth link keeps up: the front, then the
    // four other filters (NONE = no filter yet or 3+ pings without an accepted echo).
    ++polls;
    if (polls % 10 == 1) io.event(RL_F("dpoll"), i16(t), lastFront, i16(d), pwm);
    if (polls % 10 == 6) {
      int16_t s[4];
      const uint8_t order[4] = {L45, R45, LSIDE, RSIDE};
      for (uint8_t i = 0; i < 4; ++i) s[i] = live[order[i]] && blind[order[i]] < 3 ? i16(kf[order[i]].dist()) : NONE;
      io.event(RL_F("dside"), s[0], s[1], s[2], s[3]);
    }
    if (s * (d + v * stopTau - stopAt) <= 3) break;   // predicted distance after stopping
    if (z[sk] > 0 && s * (z[sk] - stopAt) < -10) break;   // a raw echo already past the target: stop, whatever the filter says
    // a corner closing on a wall at an angle: either 45 deg beam (filtered, 2+ echoes) under 100 mm
    // Corners only exist near the walls (side walls ~30 cm deep from the main wall): with the front still
    // more than CORNER_ZONE out, a close 45 deg echo is something on the open floor, not a wall (2 Nov: a
    // ~70 mm echo at the start stopped every drive and the robot backed away all run).
    bool nearWall = !live[FRONT] || kf[FRONT].dist() < CORNER_ZONE || (sk != FRONT && kf[sk].dist() < CORNER_ZONE);
    // A 45 deg echo about as far as the front's is the wall straight ahead seen by the beam's edge (3 Nov
    // sim, and the field's right box: the creep stopped ~45 mm short), not a corner: the front guards that.
    float cornerCap = live[FRONT] && !(PARK_ORIG && inPark) ? kf[FRONT].dist() - CORNER_MARGIN : 1e6f;   // the filter carries over a missed echo or two
    const int16_t rawCap = PARK_ORIG && inPark ? 32000 : int16_t(cornerMm + 30);
    // ... and only on a real close echo (3 Nov sim: a beam jumping between surfaces gave the filter a
    // speed that predicted 97 mm with every echo 400+ mm)
    if (s > 0 && (nearWall && ((live[L45] && blind[L45] < 2 && kf[L45].dist() < cornerMm && kf[L45].dist() < cornerCap && (raw45[0] > 0 || rawCap == 32000) && raw45[0] < rawCap) ||
                               (live[R45] && blind[R45] < 2 && kf[R45].dist() < cornerMm && kf[R45].dist() < cornerCap && (raw45[1] > 0 || rawCap == 32000) && raw45[1] < rawCap)))) {
      io.event(RL_F("corner_stop"), i16(kf[L45].dist()), i16(kf[R45].dist()));
      cornered = true;
      break;
    }
    if (t >= maxMs || blind[sk] >= 4) break;
  }
  if (!io.driveEnd()) return fail(RL_F("stopped"));
  int16_t after_us[NUS];
  TRY(measure(after_us));
  // Learn the stopping time from every drive, whatever ended it: after the final decision the robot moved
  // (decision distance - final reading) at about the speed |v| the filter had. (2 Nov: only distance-ended
  // drives were used, both field drives ended on the time limit and it never learned.)
  if (sk == FRONT && after_us[FRONT] > 0 && fabsf(vDecision) > 60) {
    float tauObs = s * (dDecision - after_us[FRONT]) / fabsf(vDecision);
    stopTau = clampf(0.5f * stopTau + 0.5f * tauObs, 0.01f, 0.8f);
  }
  io.event(RL_F("drive"), i16(us[sk]), after_us[sk], i16(stopAt), sk == FRONT ? i16(stopTau * 1000) : -int16_t(sk));
  if (hasBefore && io.yaw(&after)) {   // forward drives turn this robot: learn the compensation
    wcomp[0][s > 0] = int8_t(clampf(roundf(wcomp[0][s > 0] - 1.5f * (after - before) / (t > 600 ? 2.0f : 1.0f)), -40, 40));
  }
  memcpy(us, after_us, sizeof(after_us));
  return true;
}

// After a move toward a checkpoint: a timed reverse that made the error clearly worse ends the run
// (the move sizes themselves are learned in translate).
bool Runner::learn(uint8_t axis, bool hasBefore, float before, bool hasAfter, float after, uint16_t ms) {
  if (!hasBefore || !hasAfter || ms == 0) return true;   // ms == 0: a Kalman-tracked drive, not a timed pulse
  float moved = before * after > 0 ? fabsf(before) - fabsf(after) : fabsf(before) + fabsf(after);
  if (moved < -25 && axis == 0) return fail(RL_F("forward_move_went_the_wrong_way"));
  return true;   // sideways from the jumpy 45 deg beams: a bad reading, not a reason to stop
}

// Keep the gyro heading: back to the goal if a move turned the robot more than HOLD_TOL.
bool Runner::hold(int16_t us[NUS]) {
  float y;
  if (!io.yaw(&y)) return true;
  if (!goalSet) setGoal(y);
  else if (io.lost() != goalLost) {
    // Gyro samples were dropped (FIFO overflow on a long creep). Keep the heading goal (option A,
    // 2 Nov): on a straight creep the robot barely turns, so the dropped samples cost little, and
    // re-basing would throw away the heading the robot must point back to.
    io.event(RL_F("gyro_overflow_kept_goal"), i16(y * 10), i16(goal * 10));
    goalLost = io.lost();
  }
  if (fabsf(goal - y) <= HOLD_TOL) return true;
  io.event(RL_F("hold"), i16(y * 10), i16(goal * 10));
  // Near a wall (front under 150 mm) a turn swings the front corners into it (2 Nov: the left box
  // corner touched the side wall): at most 3 deg there per correction.
  if (us[FRONT] > 0 && us[FRONT] < 150) {
    TRY(rotateTo(y + clampf(goal - y, -3, 3), ROT_TOL, false));
    return measure(us);
  }
  TRY(rotateTo(goal, ROT_TOL, true));
  return measure(us);
}

// Gyro rotation, read only while stopped (no ultrasonic scan needed between pulses: the angle is
// current once a pulse has settled). Over ROT_COARSE_DEG one long smooth pulse at the coarse power
// covers ~90 %; below it fine pulses aim at the whole remaining angle. Pulse lengths come from this
// run's rotation model, so a small error gets a small turn. A pulse that does not turn makes the
// next one longer, and at full length raises the power (unknown breakaway). strict = must reach
// tol; otherwise best effort, stopping once pulses no longer get closer.
bool Runner::rotateTo(float target, float tol, bool strict, float* reached) {
  float best = -1;
  uint8_t stalls = 0;
  uint16_t lost0 = io.lost();
  for (uint8_t move = 0; move < (strict ? 20 : 8); ++move) {
    float y;
    if (!io.yaw(&y) || io.lost() != lost0) return fail(RL_F("gyro_lost_data_in_rotation"));
    float err = target - y;
    if (fabsf(err) <= tol) { if (reached) *reached = y; return true; }
    if (!strict) {
      stalls = (best >= 0 && fabsf(err) >= best - 0.3f) ? stalls + 1 : 0;
      best = best < 0 ? fabsf(err) : (fabsf(err) < best ? fabsf(err) : best);
      if (stalls >= 2) { if (reached) *reached = y; return true; }
    }
    uint8_t p = fabsf(err) > ROT_COARSE_DEG ? 0 : 1;
    float ms = rot[p].msFor(p ? fabsf(err) : 0.9f * fabsf(err));
    if (ms < rotMinMs[p]) ms = rotMinMs[p];
    uint16_t used;
    uint32_t k0 = io.ticks();
    TRY(pulse(0, 0, err > 0 ? 100 : -100, rotPwm[p], ms, &used, 40, ROT_T_MAX));
    float a;
    if (!io.yaw(&a) || io.lost() != lost0) return fail(RL_F("gyro_lost_data_in_rotation"));
    float deg = fabsf(a - y);
    uint32_t dk = io.ticks() - k0;
    if (deg >= 3 && dk > 0) tpd = tpd > 0 ? 0.7f * tpd + 0.3f * dk / deg : dk / deg;   // encoder ticks per degree
    io.event(RL_F("rotate"), err > 0 ? int16_t(used) : -int16_t(used), rotPwm[p], i16(y * 10), i16(a * 10));
    float turned = (a - y) * (err > 0 ? 1 : -1);   // in the intended direction
    if (turned < 0.3f) {   // static friction: longer next time, then more power
      rotMinMs[p] = used + 30;
      if (rotMinMs[p] > ROT_T_MAX && rotPwm[p] < 150) { rotPwm[p] += 10; rotMinMs[p] = 0; rot[p].init(rot[p].rate, rot[p].dead); }
    } else rotMinMs[p] = 0;
    rot[p].learn(used, turned > 0 ? turned : 0);
  }
  // not reached: never stop the run for it (2 Nov field: ROTATION_NOT_REACHED stopped a run at N);
  // carry on from the closest heading, logged
  float y;
  if (io.yaw(&y)) { if (reached) *reached = y; io.event(RL_F("rotation_best_effort"), i16((target - y) * 10)); }
  (void)strict;
  return true;
}

// The gentlest rotation that measurably turns the robot (>= 1.5 deg): the fine model's length for
// 2 deg, lengthened by 25 ms while it does not turn.
bool Runner::rotatePulse(int8_t dir, float y0, int16_t us[NUS], float* y1, bool* turned) {
  *turned = false;
  float ms = rot[1].msFor(2.0f);
  for (uint8_t tries = 0; tries < 8; ++tries, ms += 25) {
    uint16_t used;
    TRY(pulse(0, 0, dir * 100, rotPwm[1], ms, &used, 40, ROT_T_MAX));
    TRY(measure(us));
    if (!io.yaw(y1)) return true;
    float t = (*y1 - y0) * dir;
    rot[1].learn(used, t > 0 ? t : 0);
    if (t >= 1.5f) { *turned = true; return true; }
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
        if (fabsf(float(us1[b] - ref[b])) > 150) continue;   // far off its target: another surface, not heading (2 Nov: +25 deg turn)
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
    if (nfix == 1 && fabsf(fixes[0]) > 8) {   // one beam alone asking for a big turn: not trusted
      io.event(RL_F("beams_one_big"), i16(fixes[0] * 10));
      nfix = 0;
    }
    if (nfix == 0) {
      if (!hadGoal) { memcpy(us, us1, sizeof(int16_t) * NUS); return true; }
      setGoal(goal0);
      TRY(rotateTo(goal0, 2, false));
      return measure(us);
    }
    float fix = clampf(nfix == 2 ? (fixes[0] + fixes[1]) / 2 : fixes[0], -12, 12);   // field limit (30 Oct)
    io.event(RL_F("beams_fix"), pose, i16(slopes[0] * 10), i16(slopes[1] * 10), i16(fix * 10));
    setGoal(y1 + fix);
    if (fabsf(fix) <= 1.5f) { memcpy(us, us1, sizeof(int16_t) * NUS); return true; }
    TRY(rotateTo(goal, 1.5f, false));
    TRY(measure(us));
  }
  return true;
}

// Predicted placement accuracy at a checkpoint, 0..100 %: 100 minus the weighted rms of the
// per-sensor errors in mm (25 mm rms = 75 %). The front and side walls place the robot (weight 1);
// the 45 deg beams are noisier at N (glancing returns) and count half. An error is capped at 100 mm;
// a missing echo counts 60 mm.
static const float ACC_PROCEED = 75;   // %: good enough to go on; below it the run still goes on, flagged
float Runner::accuracy(Pose pose, const int16_t us[NUS]) const {
  const int16_t* ref = cal.ref[pose];
  float sum = 0, wsum = 0;
  for (uint8_t k = 0; k < NUS; ++k) {
    if (ref[k] == NONE) continue;
    float e = us[k] > 0 ? fabsf(float(us[k] - ref[k])) : 60;
    if (e > 100) e = 100;
    float wk = (k == L45 || k == R45) ? 0.5f : 1;
    sum += wk * e * e;
    wsum += wk;
  }
  if (wsum <= 0) return 0;
  return clampf(100 - sqrtf(sum / wsum), 0, 100);
}

// Continuous strafe that removes `frac` of the sideways offset measured on the side walls (both: half
// the difference of their readings against the capture's; one: that wall's reading), each side sensor
// tracked by its own Kalman filter while moving (~16 Hz each); the robot stops once the predicted
// offset after the stop (offset + rate * stopTauY, learned) reaches the goal. *used = false when no side
// wall gives the offset (the caller then strafes by a timed pulse).
bool Runner::driveSide(Pose pose, float frac, int16_t us[NUS], bool* used) {
  const int16_t* ref = cal.ref[pose];
  *used = false;
  bool useL = ref[LSIDE] != NONE && us[LSIDE] > 0 && fabsf(float(us[LSIDE] - ref[LSIDE])) < 450;
  bool useR = ref[RSIDE] != NONE && us[RSIDE] > 0 && fabsf(float(us[RSIDE] - ref[RSIDE])) < 450;
  if (useL && useR && fabsf(float((us[LSIDE] + us[RSIDE]) - (ref[LSIDE] + ref[RSIDE]))) > 150) {
    useL = fabsf(float(us[LSIDE] - ref[LSIDE])) < fabsf(float(us[RSIDE] - ref[RSIDE]));   // spacing wrong: trust the nearer match
    useR = !useL;
  }
  if (!useL && !useR) return true;
  *used = true;
  Kalman1D kl, kr;
  if (useL) kl.init(float(us[LSIDE]));
  if (useR) kr.init(float(us[RSIDE]));
  #define EY(l, r) (useL && useR ? (((l) - (r)) - (ref[LSIDE] - ref[RSIDE])) / 2.0f : useL ? (l) - ref[LSIDE] : ref[RSIDE] - (r))
  float e0 = EY(float(us[LSIDE]), float(us[RSIDE])), goalE = e0 * (1 - frac);
  int8_t s = e0 > 0 ? 1 : -1;   // +1 strafe left
  int8_t w = wcomp[1][s > 0];
  uint8_t pwm = inPark ? PARK_SIDE_PWM : SIDE_PWM, blindL = 0, blindR = 0;
  float before = 0, after, t = 0, tWin = 0, eDecision = e0, rDecision = 0, eStart = e0;
  bool hasBefore = io.yaw(&before);
  if (towardWall(0, s * 100)) { io.event(RL_F("wall_block"), 0, s * 100, blocked); return true; }
  if (!io.driveStart(0, s * 100, w, pwm, FOCUS_SIDES)) return fail(RL_F("stopped"));
  for (uint8_t polls = 0;; ++polls) {
    int16_t z[NUS];
    float dt;
    if (!io.drivePoll(z, &dt)) { io.driveEnd(); return fail(RL_F("stopped")); }
    t += dt * 1000;
    noteWalls(z);
    if (towardWall(0, s * 100)) { io.event(RL_F("wall_block"), 0, s * 100, blocked); break; }
    if (useL) { kl.predict(dt); if (z[LSIDE] != NOT_PINGED) { if (z[LSIDE] > 0 && kl.update(z[LSIDE])) blindL = 0; else ++blindL; } }
    if (useR) { kr.predict(dt); if (z[RSIDE] != NOT_PINGED) { if (z[RSIDE] > 0 && kr.update(z[RSIDE])) blindR = 0; else ++blindR; } }
    float e = EY(kl.dist(), kr.dist());
    float rate = useL && useR ? (kl.speed() - kr.speed()) / 2 : useL ? kl.speed() : -kr.speed();   // mm/s of offset
    if (t - tWin >= 400) {   // not moving at all over 400 ms: a little more power (no lunges)
      if (s * (eStart - e) < 3 && pwm < 110) {
        pwm += 4;
        if (!io.driveSet(0, s * 100, w, pwm)) { io.driveEnd(); return fail(RL_F("stopped")); }
      }
      eStart = e;
      tWin = t;
    }
    eDecision = e;
    rDecision = rate;
    if (polls % 8 == 1) io.event(RL_F("spoll"), i16(t), i16(e), i16(rate), pwm);
    if (t >= 150 && s * (e + rate * stopTauY - goalE) <= 2) break;   // not before the speed estimate settles
    if (t >= 2000 || (useL && blindL >= 4) || (useR && blindR >= 4)) break;
  }
  #undef EY
  if (!io.driveEnd()) return fail(RL_F("stopped"));
  int16_t was[NUS];
  memcpy(was, us, sizeof(was));
  TRY(measure(us));
  bool okL = !useL || us[LSIDE] > 0, okR = !useR || us[RSIDE] > 0;
  if (okL && okR && fabsf(rDecision) > 30) {
    float l = useL ? us[LSIDE] : 0, r = useR ? us[RSIDE] : 0;
    float eAfter = useL && useR ? ((l - r) - (ref[LSIDE] - ref[RSIDE])) / 2.0f : useL ? l - ref[LSIDE] : ref[RSIDE] - r;
    float tauObs = s * (eDecision - eAfter) / fabsf(rDecision);
    stopTauY = clampf(0.5f * stopTauY + 0.5f * tauObs, 0.01f, 0.5f);
  }
  io.event(RL_F("strafe"), i16(e0), i16(goalE), i16(stopTauY * 1000), pwm);
  if (hasBefore && io.yaw(&after)) {   // strafes turn this robot: learn the compensation
    wcomp[1][s > 0] = int8_t(clampf(roundf(w - 1.6f * (after - before)), -60, 60));   // 2 Nov: strafes turned 3-7 deg
  }
  (void)was;
  return true;
}

// Settle on a checkpoint with damped corrections, a PD-like loop on the measured offsets: forward/back
// from the front sensor by a continuous Kalman-tracked drive (either direction), sideways from the side
// walls by a model-sized strafe, each covering `gain` of the error. A correction that overshoots (the
// error changed sign) lowers that axis' gain, so the robot closes in instead of going in and out
// (2 Nov field run); inside DEAD_MM an axis is left alone. Done at ACC_GOOD, when both axes are inside
// the dead band, when the accuracy stops improving above ACC_PROCEED, or after `moves` corrections:
// it then proceeds and reports the predicted accuracy. Never an error for an imperfect checkpoint.
bool Runner::settle(Pose pose, uint8_t moves, int16_t us[NUS], float target, float dead) {
  float gain[2] = {0.9f, 0.9f}, lastErr[2] = {0, 0}, best = -1;
  uint8_t noGain = 0;
  TRY(measure(us));
  for (uint8_t move = 0;; ++move) {
    float ex, ey;
    bool hx, hy;
    errors(pose, us, true, &ex, &hx, &ey, &hy);   // sideways from the side walls, else the 45 deg pair
    float acc = accuracy(pose, us);
    noGain = acc > best + 1 ? 0 : noGain + 1;
    if (acc > best) best = acc;
    io.event(RL_F("settle"), pose * 100 + move, hx ? i16(ex) : NONE, hy ? i16(ey) : NONE, i16(acc));
    bool xok = !hx || fabsf(ex) <= dead, yok = !hy || fabsf(ey) <= dead;
    bool inBand = hx && hy && xok && yok;   // both axes measured and close: only then is the band "done"
    if (acc >= target || inBand || move >= moves || (noGain >= 3 && acc >= ACC_PROCEED)) {
      io.event(RL_F("settled"), pose, i16(acc), i16(best), acc >= ACC_PROCEED ? 1 : 0);
      return true;
    }
    uint8_t axis = !xok && (yok || fabsf(ex) >= fabsf(ey)) ? 0 : 1;
    float err = axis ? ey : ex;
    if (lastErr[axis] != 0 && (err > 0) != (lastErr[axis] > 0)) gain[axis] = clampf(0.6f * gain[axis], 0.35f, 1);  // overshoot: damp
    else if (lastErr[axis] != 0 && fabsf(err) > 0.6f * fabsf(lastErr[axis])) gain[axis] = clampf(1.15f * gain[axis], 0.35f, 1);  // too timid
    lastErr[axis] = err;
    if (axis == 0) TRY(translate(0, gain[0] * ex / 0.9f, us));   // translate's drive covers 90 %
    else {
      bool used;
      TRY(driveSide(pose, gain[1], us, &used));                     // continuous, on the side walls
      if (!used) TRY(translate(1, gain[1] * ey / FRACTION, us));    // no side wall: timed strafe (45s)
    }
    TRY(hold(us));
  }
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
      if (++lost > 2) { io.event(RL_F("seek_front_lost_continue"), pose, move); return true; }   // proceed, do not give up
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
      if (lostSide++ >= 2) {
        // The side wall is out of sight (heading off the wall's normal, 2 Nov): go on with the front
        // sensor alone rather than abort the whole run.
        io.event(RL_F("side_unseen_continue"), pose, move);
        return true;
      }
      TRY(measure(us));
      continue;
    } else {
      io.event(RL_F("reached_no_sideways"), pose, move);
      return true;
    }
    memcpy(us, us2, sizeof(us2));
  }
  io.event(RL_F("seek_best_effort"), pose, i16(accuracy(pose, us)));   // proceed, do not give up
  return true;
}

// Match a checkpoint on every sensor it has a target for (N: all five; P: front and the 45s):
// mismatch = sum of |reading - target|, each capped at 300 mm (a missing echo counts 300), so one
// sensor seeing something else cannot dominate. Forward/back directly from the front sensor;
// sideways and heading by trial moves (strafe left/right, turn on the gyro) kept only if the
// mismatch drops by > 8 mm; steps shrink when nothing helps. Done at ~25 mm a sensor; otherwise
// stop rather than go on from the wrong place.
bool Runner::descend(Pose pose, bool turns, int16_t us[NUS], bool mustReach) {
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
  for (uint8_t move = 0; move < (mustReach ? 40 : 10); ++move) {   // best effort: only centring for the sweep
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
  if (!mustReach) { io.event(RL_F("match_best_effort"), pose, i16(cost)); return true; }
  return fail(RL_F("checkpoint_match_not_reached"));
}

// Start: straight at the main wall on the held start heading (the robot is placed facing it) until
// the front reads N's front distance, in slow Kalman-tracked drives (each stops at 90 % of the way
// left). There is no P checkpoint (deleted 2 Nov): the N match takes over from where this ends.
bool Runner::approach(int16_t us[NUS]) {
  for (uint8_t t = 0; t < 3; ++t) { TRY(measure(us)); if (us[FRONT] > 0) break; }
  if (us[FRONT] <= 0) return fail(RL_F("approach_front_sees_nothing"));
  int16_t target = cal.ref[N][FRONT];
  io.event(RL_F("approach"), us[FRONT], target);
  for (uint8_t move = 0; move < 20; ++move) {
    if (us[FRONT] > 0 && us[FRONT] - target <= 25) return true;
    if (us[FRONT] <= 0) { TRY(measure(us)); continue; }
    TRY(translate(0, us[FRONT] - target, us));
    TRY(hold(us));
  }
  return true;   // best effort: the N match takes over from wherever it got
}

// cos for the wall-response fit (series, accurate to ~1e-4 up to 75 deg; no libm cos on the Mega:
// the fit evaluates it tens of thousands of times).
static float cosApprox(float a) {
  float a2 = a * a;
  return 1 - a2 * (0.5f - a2 * (0.0416667f - a2 * (0.0013889f - a2 * 0.0000248f)));
}

// How well "wall normal at psi0" explains a sweep. The front sensor sits FRONT_FWD ahead of the
// turning centre, so at a turn delta = yaw - psi0 it is (D - FRONT_FWD cos delta) from a flat wall,
// where D is the centre's distance; a beam of half-width c reads that while the normal is inside the
// beam, and that over cos(|delta| - c) beyond. D is the median over the readings taken with the
// normal inside the beam (a closer side wall at the sweep's edge cannot set it); residuals are capped
// at 40 mm, so readings off a side wall or another surface cannot dominate. Fewer than two readings
// inside the beam: no fit (huge cost).
static const float FRONT_FWD = 150;   // mm, front sensor ahead of the turning centre
static const float DEG = 0.01745329f;
static float wallCost(const float* yaws, const int16_t* fronts, uint8_t n, float psi0, float c, float* D) {
  float in[40];
  uint8_t m = 0;
  for (uint8_t i = 0; i < n && m < 40; ++i) {
    float dd = yaws[i] - psi0;
    if (fabsf(dd) <= c) {
      float f = fronts[i] + FRONT_FWD * cosApprox(dd * DEG);
      uint8_t j = m++;
      while (j > 0 && in[j - 1] > f) { in[j] = in[j - 1]; --j; }
      in[j] = f;
    }
  }
  if (m < 2) return 1e9f;
  float d = m % 2 ? in[m / 2] : 0.5f * (in[m / 2 - 1] + in[m / 2]);
  float cost = 0;
  for (uint8_t i = 0; i < n; ++i) {
    float dd = yaws[i] - psi0;
    float perp = d - FRONT_FWD * cosApprox(dd * DEG);
    float off = fabsf(dd) - c;
    float model = off > 0 ? perp / cosApprox(off * DEG) : perp;
    float r = fabsf(fronts[i] - model);
    cost += r < 40 ? r : 40;
  }
  *D = d;
  return cost;
}

// Best wall normal for a sweep: psi0 over the sweep in 2 deg steps and the beam half-width 6..15 deg,
// then refined to 0.25 deg.
static float fitWall(const float* yaws, const int16_t* fronts, uint8_t n, float* psiOut, float* cOut, float* dOut) {
  float ylo = yaws[0], yhi = yaws[0];
  for (uint8_t i = 1; i < n; ++i) { if (yaws[i] < ylo) ylo = yaws[i]; if (yaws[i] > yhi) yhi = yaws[i]; }
  float bestCost = 1e9f, bestPsi = 0.5f * (ylo + yhi), bestC = 10, bestD = 0, d;
  for (float c = 6; c <= 15.01f; c += 3)
    for (float psi = ylo; psi <= yhi; psi += 2.0f) {
      float cost = wallCost(yaws, fronts, n, psi, c, &d);
      if (cost < bestCost) { bestCost = cost; bestPsi = psi; bestC = c; bestD = d; }
    }
  for (float step = 0.5f; step >= 0.24f; step *= 0.5f) {
    float p0 = bestPsi, c0 = bestC;
    for (float psi = p0 - 4 * step; psi <= p0 + 4.01f * step; psi += step)
      for (float c = c0 - 1.5f; c <= c0 + 1.51f; c += 0.75f) {
        float cost = wallCost(yaws, fronts, n, psi, c, &d);
        if (cost < bestCost) { bestCost = cost; bestPsi = psi; bestC = c; bestD = d; }
      }
  }
  *psiOut = bestPsi; *cOut = bestC; *dOut = bestD;
  return bestCost;
}

// Smooth sweep from where the robot is to `end` (gyro deg): one continuous rotation at the fine power,
// its length from the learned sweep rate; a top-up sweep if it stopped more than 8 deg short.
bool Runner::sweepTo(float end, float* yaws, int16_t* fronts, uint8_t* n, uint8_t maxN) {
  for (uint8_t leg = 0; leg < 3 && *n < maxN; ++leg) {
    float y, y1;
    if (!io.yaw(&y)) return fail(RL_F("gyro_lost_data_in_sweep"));
    float left = end - y;
    if (fabsf(left) < 8) return true;
    int8_t dir = left > 0 ? 1 : -1;
    float ms = clampf(fabsf(left) / sweepRate + 60, 150, 1900);   // + the soft start
    uint8_t got = 0;
    if (!io.sweep(dir, rotPwm[1], uint16_t(ms), yaws + *n, fronts + *n, maxN - *n, &got)) return fail(RL_F("sweep_stopped_or_gyro_lost"));
    *n += got;
    if (!io.yaw(&y1)) return fail(RL_F("gyro_lost_data_in_sweep"));
    float turned = (y1 - y) * dir;
    if (turned > 5) sweepRate = clampf(0.5f * sweepRate + 0.5f * turned / ms, 0.02f, 0.5f);
    else if (rotPwm[1] < 150) rotPwm[1] += 10;   // did not turn: more power
  }
  return true;
}

// Square to the main wall where the robot is. One smooth turn to -half deg, one smooth continuous
// sweep to +half deg (70 at N; 40 when parking, close to a wall) with the front sensor pinged all the way (~1 deg apart), then fit the wall response
// (fitWall): the normal is the psi0 that explains the readings best. Side-wall readings at the sweep's
// edges only cost the capped residual. Then one smooth turn onto the normal. (The step sweep this
// replaces stopped ~20 times and was off by 4.4 deg on average in the 2 Nov simulation.)
bool Runner::squareHere(float half) {
  float y0;
  if (!io.yaw(&y0)) return fail(RL_F("gyro_not_valid"));
  static float yaws[90];
  static int16_t fronts[90];
  uint8_t n = 0;
  TRY(rotateTo(y0 - half, 4, false));
  TRY(sweepTo(y0 + half, yaws, fronts, &n, 90));
  uint8_t m = 0;   // keep the pings that had an echo
  for (uint8_t i = 0; i < n; ++i) {
    if (i % 3 == 0) io.event(RL_F("sweep"), i16((yaws[i] - y0) * 10), fronts[i]);
    if (fronts[i] > 0) { yaws[m] = yaws[i]; fronts[m] = fronts[i]; ++m; }
  }
  if (m < 8) return fail(RL_F("sweep_saw_no_wall"));
  float psi, c, d;
  float cost = fitWall(yaws, fronts, m, &psi, &c, &d);
  io.event(RL_F("square"), i16((psi - y0) * 10), i16(d), i16(c * 10), i16(cost / m * 10));
  setGoal(psi);
  return rotateTo(psi, ROT_TOL, false);
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

// Final approach, no rotation near the wall: Kalman-tracked slow drive to 150 mm short of the box's front
// reading, hold the heading and square on the box's 45 deg beam from there (>= 15 cm from the wall,
// so turning cannot touch it), then a constant slow creep to the front reading (driveForward stops
// by predicted distance). Backs off if already closer than 12 mm to the target; never drives without
// a front echo. 2 Nov: fixed-length moves went 161 -> 55 -> 29 mm and touched the wall.
bool Runner::park(Pose pose, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  int8_t side = ref[RSIDE] != NONE ? RSIDE : ref[LSIDE] != NONE ? LSIDE : -1;
  TRY(measure(us));
  int16_t prev = -1;
  bool squared = false;
  uint8_t blind = 0;
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
      if (++blind > 4) return fail(RL_F("park_front_echo_lost"));
      prev = -1;
      if (blind == 3) {   // no echo twice in a row: the wall reflects the beam away (heading > ~10 deg off): re-find it
        io.event(RL_F("park_refind_wall"), move);
        TRY(squareHere(40));
      }
      TRY(measure(us));
      continue;
    }
    blind = 0;
    prev = front;
    float toGo = front - ref[FRONT];
    if (toGo < -12) {   // too close: back straight off
      TRY(translate(0, toGo, us));
      continue;
    }
    // Sideways only while >= 10 cm from the wall (a strafe also turns the robot); tighter once squared.
    if (hasEy && fabsf(ey) > (squared ? 15.0f : 30.0f) && toGo > 100) {
      TRY(translate(1, ey, us));
      TRY(hold(us));
      continue;
    }
    if (toGo > 180) {
      TRY(driveForward(ref[FRONT] + 150, DRIVE_MAX_MS, DRIVE_PWM, us));
      TRY(hold(us));
      continue;
    }
    if (!squared) {
      squared = true;
      if (toGo >= 60) {
        TRY(hold(us));
        TRY(squareToBeams(pose, us));
        continue;
      }
    }
    if (toGo > 80) {   // creep to 6 cm short, then re-check the heading while there is still room to turn
      TRY(driveForward(ref[FRONT] + 60, DRIVE_MAX_MS, DRIVE_PWM, us));
      TRY(hold(us));
      continue;
    }
    if (toGo > 8) {   // the last stretch: straight, no rotation this close to the wall
      TRY(driveForward(ref[FRONT], DRIVE_MAX_MS, DRIVE_PWM, us));
      continue;
    }
    float y;
    bool hy = io.yaw(&y);
    io.event(RL_F("parked"), front, ref[FRONT], hy && goalSet ? i16((y - goal) * 10) : NONE);
    return true;
  }
  return fail(RL_F("park_not_finished"));
}

bool Runner::creepTest(int16_t target) {
  int16_t us[NUS];
  TRY(measure(us));
  for (uint8_t i = 0; i < 8; ++i) {
    if (us[FRONT] <= 0) return fail(RL_F("no_front_echo"));
    if (us[FRONT] - target <= 8) { io.event(RL_F("creep_done"), us[FRONT], target); return true; }
    TRY(driveForward(target, DRIVE_MAX_MS, DRIVE_PWM, us));
  }
  io.event(RL_F("creep_done"), us[FRONT], target);
  return true;
}

// ---- Position correction (2 Nov field plan) -------------------------------------------------
// Sensor sets: N uses all five; after a right turn (and its box) front, front-left (L45) and left
// side; after a left turn front, front-right (R45) and right side.
TUNE(uint8_t) FIX_PWM = 68;   // combined moves (strafe needs about this much)
static const float FIX_DEAD = 70;    // ms of soft start before a move really moves

// Accuracy over a sensor set, 0..100 %: 100 - rms error in mm (cap 100, missing echo 60). The 45 deg
// beams count a quarter (noisy glancing and corner returns); front and side walls place the robot.
float Runner::accuracySet(Pose pose, uint8_t set, const int16_t us[NUS]) const {
  const int16_t* ref = cal.ref[pose];
  float sum = 0, wsum = 0;
  for (uint8_t k = 0; k < NUS; ++k) {
    bool in = set == SET_ALL || k == FRONT || (set == SET_RIGHT_TURN ? (k == L45 || k == LSIDE) : (k == R45 || k == RSIDE));
    if (!in || ref[k] == NONE) continue;
    float e = us[k] > 0 ? fabsf(float(us[k] - ref[k])) : 60;
    if (e > 100) e = 100;
    float wk = (k == L45 || k == R45) ? 0.25f : 1;
    sum += wk * e * e;
    wsum += wk;
  }
  return wsum > 0 ? clampf(100 - sqrtf(sum / wsum), 0, 100) : 0;
}

// Fresh readings; near the target two scans averaged (sensor noise ~4 mm limits a 98 % target).
bool Runner::measureFine(int16_t us[NUS], bool twice) {
  TRY(measure(us));
  if (!twice) return true;
  int16_t b[NUS];
  TRY(measure(b));
  for (uint8_t k = 0; k < NUS; ++k)
    if (us[k] > 0 && b[k] > 0 && fabsf(float(us[k] - b[k])) < 30) us[k] = (us[k] + b[k] + 1) / 2;
    else if (us[k] <= 0) us[k] = b[k];
  return true;
}

// Simple feedback loop to a calibrated pose, one combined move per round:
//  1 measure the stage's sensors; accuracy >= target: done
//  2 position error: forward from the front, sideways from the side sensor(s); heading error from the
//    gyro goal, which the 45 deg beam refines once the position is close (its mm per deg measured by
//    one ~2 deg probe turn at the start of the stage)
//  3 one mecanum move doing forward + sideways + rotation together, each covering `gain` of its error,
//    lengths from rates learned from every move's actual result; a gain drops after an overshoot
//  4 repeat; after maxMoves the run goes on from there (logged), never an error.
// No creeping: every move is a short timed move, measured while still.
bool Runner::fixPose(Pose pose, uint8_t set, float target, uint8_t maxMoves, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  uint8_t k45 = set == SET_RIGHT_TURN ? L45 : R45;
  // 45 deg beam slope (mm per deg CCW): one small probe turn. At N the difference L45 - R45 is used.
  float slope = 0;
  {
    float y0, y1;
    int16_t a[NUS], b[NUS];
    TRY(measure(a));
    bool turned;
    if (io.yaw(&y0)) {
      TRY(rotatePulse(+1, y0, b, &y1, &turned));
      if (turned && fabsf(y1 - y0) > 1) {
        float da = set == SET_ALL ? float((b[L45] - b[R45]) - (a[L45] - a[R45])) : float(b[k45] - a[k45]);
        bool ok = set == SET_ALL ? (a[L45] > 0 && a[R45] > 0 && b[L45] > 0 && b[R45] > 0) : (a[k45] > 0 && b[k45] > 0);
        if (ok) slope = da / (y1 - y0);
        if (fabsf(slope) < 3 || fabsf(slope) > 60) slope = 0;   // flat or flipping across an edge: gyro only
      }
    }
    io.event(RL_F("fix_probe"), pose, set, i16(slope * 10));
  }
  float gain[3] = {0.8f, 0.8f, 0.8f}, last[3] = {0, 0, 0}, best = 0;
  for (uint8_t move = 0;; ++move) {
    TRY(measureFine(us, best > target - 10));
    float acc = accuracySet(pose, set, us);
    if (acc > best) best = acc;
    // position errors (+ex: drive forward, +ey: strafe left)
    float ex = 0, ey = 0;
    bool hx = ref[FRONT] != NONE && us[FRONT] > 0, hy = false;
    if (hx) ex = float(us[FRONT] - ref[FRONT]);
    if (set == SET_ALL) {
      bool l = ref[LSIDE] != NONE && us[LSIDE] > 0, r = ref[RSIDE] != NONE && us[RSIDE] > 0;
      if (l && r) { ey = ((us[LSIDE] - us[RSIDE]) - (ref[LSIDE] - ref[RSIDE])) / 2.0f; hy = true; }
      else if (l) { ey = float(us[LSIDE] - ref[LSIDE]); hy = true; }
      else if (r) { ey = float(ref[RSIDE] - us[RSIDE]); hy = true; }
    } else if (set == SET_RIGHT_TURN) {
      if (ref[LSIDE] != NONE && us[LSIDE] > 0) { ey = float(us[LSIDE] - ref[LSIDE]); hy = true; }
    } else if (ref[RSIDE] != NONE && us[RSIDE] > 0) { ey = float(ref[RSIDE] - us[RSIDE]); hy = true; }
    // heading: the 45 deg beam refines the gyro goal once the position is close
    float e45 = NONE;
    if (slope != 0 && fabsf(ex) < 12 && fabsf(ey) < 12) {
      if (set == SET_ALL && us[L45] > 0 && us[R45] > 0) e45 = float((us[L45] - us[R45]) - (ref[L45] - ref[R45]));
      else if (set != SET_ALL && us[k45] > 0 && ref[k45] != NONE) e45 = float(us[k45] - ref[k45]);
      if (e45 != NONE && fabsf(e45) > 4) goal += clampf(-0.7f * e45 / slope, -3, 3);
    }
    float yaw;
    if (!io.yaw(&yaw)) return fail(RL_F("gyro_not_valid"));
    if (io.lost() != goalLost) { setGoal(yaw); }   // gyro data lost: re-base, the beam refines it again
    float eth = goal - yaw;
    io.event(RL_F("fix"), pose * 100 + move, hx ? i16(ex) : NONE, hy ? i16(ey) : NONE, i16(acc));
    if (acc >= target || move >= maxMoves) {
      io.event(RL_F("fixed"), pose, i16(acc), i16(best), acc >= target ? 1 : 0);
      return true;
    }
    float err[3] = {hx ? ex : 0, hy ? ey : 0, eth};
    for (uint8_t i = 0; i < 3; ++i) {
      if (last[i] != 0 && err[i] != 0 && (err[i] > 0) != (last[i] > 0)) gain[i] = clampf(0.6f * gain[i], 0.3f, 1);
      else if (last[i] != 0 && fabsf(err[i]) > 0.6f * fabsf(last[i])) gain[i] = clampf(1.1f * gain[i], 0.3f, 1);
      last[i] = err[i];
    }
    float dx = gain[0] * err[0], dy = gain[1] * err[1], dth = gain[2] * err[2];
    // limits: forward at most 150 mm a move, 30 mm once close to the wall, never past the target;
    // no forward move with a front corner (45 deg beam) under 70 mm
    float room = hx ? ex : 0;
    if (dx > 0) dx = clampf(dx, 0, us[FRONT] < ref[FRONT] + 80 ? 30 : 150) < room ? clampf(dx, 0, us[FRONT] < ref[FRONT] + 80 ? 30 : 150) : room;
    if (dx > 0 && ((us[L45] > 0 && us[L45] < 70) || (us[R45] > 0 && us[R45] < 70))) dx = 0;
    dx = clampf(dx, -100, 150);
    dy = clampf(dy, -120, 120);
    dth = clampf(dth, -15, 15);
    if (fabsf(dx) < 3) dx = 0;
    if (fabsf(dy) < 3) dy = 0;
    if (fabsf(dth) < 0.7f) dth = 0;
    if (dx == 0 && dy == 0 && dth == 0) {   // nothing left the loop can act on
      io.event(RL_F("fixed"), pose, i16(acc), i16(best), acc >= target ? 1 : 0);
      return true;
    }
    if (dx == 0 && dy == 0) { TRY(rotateTo(goal, ROT_TOL, false)); continue; }   // rotation only: the slow turn
    // one combined move: time each axis needs at full rate; the slowest runs full, the rest scaled
    float c[3] = {fabsf(dx) / fixRate[0], fabsf(dy) / fixRate[1], fabsf(dth) / fixRate[2]};
    float cmax = c[0] > c[1] ? c[0] : c[1];
    if (c[2] > cmax) cmax = c[2];
    int8_t mx = int8_t((dx > 0 ? 1 : -1) * roundf(100 * c[0] / cmax));
    int8_t my = int8_t((dy > 0 ? 1 : -1) * roundf(100 * c[1] / cmax));
    int8_t mw = int8_t((dth > 0 ? 1 : -1) * roundf(100 * c[2] / cmax));
    float T = clampf(cmax + FIX_DEAD, 60, 900);
    int16_t was[NUS];
    memcpy(was, us, sizeof(was));
    float y0;
    io.yaw(&y0);
    uint16_t used;
    TRY(pulse(mx, my, mw, FIX_PWM, T, &used, 60, 900));
    io.event(RL_F("fix_move"), i16(dx), i16(dy), i16(dth * 10), int16_t(used));
    // learn the rates from what the move really did (front, side sensor(s), gyro)
    int16_t now[NUS];
    TRY(measure(now));
    float run = used - FIX_DEAD;
    float obs[3] = {0, 0, 0};
    bool have[3] = {false, false, false};
    if (was[FRONT] > 0 && now[FRONT] > 0 && fabsf(float(now[FRONT] - was[FRONT])) < 300) { obs[0] = float(was[FRONT] - now[FRONT]); have[0] = true; }
    if (set != SET_LEFT_TURN && was[LSIDE] > 0 && now[LSIDE] > 0 && fabsf(float(now[LSIDE] - was[LSIDE])) < 300) { obs[1] = float(was[LSIDE] - now[LSIDE]); have[1] = true; }
    else if (set != SET_RIGHT_TURN && was[RSIDE] > 0 && now[RSIDE] > 0 && fabsf(float(now[RSIDE] - was[RSIDE])) < 300) { obs[1] = float(now[RSIDE] - was[RSIDE]); have[1] = true; }
    float y1;
    if (io.yaw(&y1)) { obs[2] = y1 - y0; have[2] = true; }
    const int8_t m3[3] = {mx, my, mw};
    for (uint8_t i = 0; i < 3; ++i) {
      float eff = fabsf(m3[i]) / 100.0f * run;
      if (!have[i] || eff < 25 || obs[i] * m3[i] <= 0) continue;
      float r = fabsf(obs[i]) / eff;
      fixRate[i] = clampf(0.6f * fixRate[i] + 0.4f * r, fixRate0[i] / 4, fixRate0[i] * 4);
    }
    memcpy(us, now, sizeof(now));
  }
}

// Straighten at N from the two side walls (2 Nov field: the 45 deg beams gave wrong or no heading,
// and a 25-30 deg start error survived to the box). A side sensor reads its wall's perpendicular
// distance while the wall's normal is inside its beam (half-width c), D / cos(|yaw - psi| - c) beyond;
// both side walls are parallel, so both share the straight heading psi. Readings at 5 headings
// (-20, -10, 0, +10, +20 deg, measured while still), then psi and c fitted over both sides
// (residuals capped at 60 mm); one turn onto psi. A poor fit keeps the heading.
static float sideCost(const float* yaws, const int16_t* v, uint8_t n, float psi, float c) {
  int16_t d = 32767;
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; ++i) if (v[i] > 0) { if (v[i] < d) d = v[i]; ++m; }
  if (m < 3) return -1;
  float cost = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (v[i] <= 0) continue;
    float off = fabsf(yaws[i] - psi) - c;
    float model = off > 0 ? d / cosApprox(off * DEG) : d;
    float r = fabsf(v[i] - model);
    cost += r < 60 ? r : 60;
  }
  return cost / m;
}

bool Runner::squareToSides() {
  float y0;
  if (!io.yaw(&y0)) return true;
  float yaws[5];
  int16_t ls[5], rs[5], us[NUS];
  const float off[5] = {-20, -10, 0, 10, 20};
  for (uint8_t i = 0; i < 5; ++i) {
    TRY(rotateTo(y0 + off[i], 1.5f, false));
    TRY(measure(us));
    if (!io.yaw(&yaws[i])) return true;
    ls[i] = us[LSIDE];
    rs[i] = us[RSIDE];
    io.event(RL_F("side_look"), i16((yaws[i] - y0) * 10), ls[i], rs[i]);
  }
  float best = 1e9f, bestPsi = y0, bestC = 8;
  for (float c = 4; c <= 12.01f; c += 2)
    for (float psi = y0 - 40; psi <= y0 + 40.01f; psi += 0.5f) {
      float a = sideCost(yaws, ls, 5, psi, c), b = sideCost(yaws, rs, 5, psi, c);
      float cost = a >= 0 && b >= 0 ? (a + b) / 2 : a >= 0 ? a : b;
      if (cost >= 0 && cost < best) { best = cost; bestPsi = psi; bestC = c; }
    }
  io.event(RL_F("side_square"), i16((bestPsi - y0) * 10), i16(bestC * 10), i16(best * 10));
  if (best > 25) {   // the side walls do not explain the readings: keep the heading
    TRY(rotateTo(y0, 1.5f, false));
    return true;
  }
  setGoal(bestPsi);
  return rotateTo(bestPsi, ROT_TOL, false);
}

// The colour turn in one smooth motion: a single coarse pulse sized by the rotation model for the
// whole 90 deg, then a fine correction only if it ended more than 3 deg off.
bool Runner::turnOnce(bool ccw) {
  float start, y, a;
  if (goalSet && io.lost() == goalLost) start = goal;
  else if (!io.yaw(&start)) return fail(RL_F("gyro_not_valid_for_the_turn"));
  setGoal(start + (ccw ? 90 : -90));
  if (!io.yaw(&y)) return fail(RL_F("gyro_not_valid_for_the_turn"));
  float want = fabsf(goal - y);
  uint16_t used;
  if (tpd > 0) {
    // Wheel encoders (user, 2 Nov): one rotation stopped by the wheel ticks for 95 % of the turn (ticks per
    // degree learned from this run's gyro-checked turns, minus the ticks it coasts after a stop); the gyro
    // check below finishes it.
    float target = tpd * 0.95f * want - coast;
    uint32_t got;
    if (!io.turnTicks(ccw ? 1 : -1, rotPwm[0], uint32_t(target > 10 ? target : 10), 2200, &got)) return fail(RL_F("stopped"));
    if (io.yaw(&a)) {
      float t = (a - y) * (ccw ? 1 : -1);
      io.event(RL_F("enc_turn"), i16(target), i16(got), i16(t * 10), i16(tpd * 10));
      if (t > 20 && got > 0) {
        coast = clampf(0.5f * coast + 0.5f * (got - (target > 10 ? target : 10)), 0, 2000);
        tpd = 0.5f * tpd + 0.5f * got / t;
      }
    }
  } else {
    // no encoder data (none counted on this run's turns): one smooth timed pulse for ~85 % of the turn
    io.event(RL_F("enc_unknown"));
    TRY(pulse(0, 0, ccw ? 100 : -100, rotPwm[0], rot[0].msFor(0.85f * want), &used, 40, ROT_T_MAX));
    if (io.yaw(&a)) { float t = (a - y) * (ccw ? 1 : -1); rot[0].learn(used, t > 0 ? t : 0); }
  }
  // check the turn on the gyro and correct it (slow fine turns) until within 0.8 deg
  if (io.yaw(&a) && fabsf(goal - a) > ROT_TOL) TRY(rotateTo(goal, ROT_TOL, true, &a));
  io.event(RL_F("turned"), ccw, i16((a - start) * 10));
  return true;
}

// Creep forward until the front reads `target` (+-15 mm): one continuous slow drive that slows as it
// closes in (driveForward), again if it stopped short; the heading is held between creeps while clear
// of the wall.
bool Runner::walkTo(int16_t target, int16_t us[NUS]) {
  for (uint8_t t = 0; t < 3 && us[FRONT] <= 0; ++t) TRY(measure(us));
  for (uint8_t move = 0; move < 12; ++move) {
    if (us[FRONT] <= 0) { TRY(measure(us)); if (us[FRONT] <= 0) return true; continue; }
    if (us[FRONT] - target <= 15) return true;
    if (us[FRONT] > 1500) { io.event(RL_F("no_wall_ahead"), us[FRONT]); return true; }   // the field is 1.5 m wide
    TRY(driveForward(target, CREEP_MAX_MS, DRIVE_PWM, us));
    if (cornered) return true;   // stopped at a wall: never creep on toward it (2 Nov: 12 restarts, 45 deg at 47 mm)
    if (us[FRONT] > 150) TRY(hold(us));
  }
  return true;
}

// ---- 2 Nov game plan: creep, straight back to N, combined omnidirectional moves ----------------
TUNE(float) PARK_DEAD_Y = 10;    // mm along the side wall left in the box (3 Nov sim: 5 / 8 mm strafed +6 -20 +10, +15 -9 +10 -6; 10 beats the team on every park metric)
TUNE(float) HEADING_MOVE = 110;
TUNE(float) N_SKEW_MAX = 12;            // deg: largest heading fix from one heading move at N
TUNE(float) PARK_APPROACH = 180;        // mm short of the box's front reading where sideways is settled
TUNE(float) PARK_SKEW_MAX = 8;          // deg: heading fix after the walk toward the box
TUNE(float) PARK_FINAL_SKEW_MAX = 3;    // deg: heading fix after the creep into the box
TUNE(float) PARK_RECOVER_MAX = 5;       // deg: heading fix after backing off a wall in the box   // mm toward the main wall from N and back: inside the side walls
TUNE(int16_t) APPROACH_MM = 500;   // stage 1: creep until the front reads ~50 cm (2 Nov field: 20 cm hit the wall)
static const float STRAFE_EFF = 0.6f;     // sideways mm per forward mm at the same wheel power (rollers slip)
static const float MOVE_MAX_MS = 2400;    // one combined move stays inside the gyro FIFO (yaw only: 2.5 s)

// Position error against a calibrated pose from readings d (mm, <= 0 = none): ex (+ = drive forward)
// from the front; ey (+ = strafe left) from the stage's side wall(s): at N both (half the difference,
// so a heading error cancels; a spacing unlike the capture's means one sees another surface: the nearer
// match is kept), after a turn and in the box the one facing the main wall.
static void posErr(const int16_t* ref, uint8_t set, const float* d, float* ex, bool* hx, float* ey, bool* hy) {
  *hx = ref[FRONT] != NONE && d[FRONT] > 0;
  *ex = *hx ? d[FRONT] - ref[FRONT] : 0;
  bool l = set != SET_LEFT_TURN && ref[LSIDE] != NONE && d[LSIDE] > 0 && fabsf(d[LSIDE] - ref[LSIDE]) < 400;
  bool r = set != SET_RIGHT_TURN && ref[RSIDE] != NONE && d[RSIDE] > 0 && fabsf(d[RSIDE] - ref[RSIDE]) < 400;
  if (l && r && fabsf((d[LSIDE] + d[RSIDE]) - (ref[LSIDE] + ref[RSIDE])) > 120) {
    if (fabsf(d[LSIDE] - ref[LSIDE]) < fabsf(d[RSIDE] - ref[RSIDE])) r = false;
    else l = false;
  }
  *hy = l || r;
  *ey = l && r ? ((d[LSIDE] - d[RSIDE]) - (ref[LSIDE] - ref[RSIDE])) / 2 : l ? d[LSIDE] - ref[LSIDE] : r ? ref[RSIDE] - d[RSIDE] : 0;
}

static void toF(const int16_t us[NUS], float d[NUS]) {
  for (uint8_t k = 0; k < NUS; ++k) d[k] = us[k];
}

// Heading from a straight forward/back move, no sweep: a robot turned theta (CCW) off square that
// moves b along its own axis changes the front reading by b cos theta and the perpendicular side-wall
// distances by b sin theta (backing up: left grows, right shrinks), so theta = atan(side change /
// front change), whatever the sideways offset. false: no usable side wall at both ends, or too short.
static bool skewFrom(uint8_t set, const int16_t* a, const int16_t* b, float* deg) {
  float dF = float(b[FRONT] - a[FRONT]);
  if (a[FRONT] <= 0 || b[FRONT] <= 0 || fabsf(dF) < 50) return false;
  bool l = set != SET_LEFT_TURN && a[LSIDE] > 0 && b[LSIDE] > 0 && fabsf(float(b[LSIDE] - a[LSIDE])) < 0.7f * fabsf(dF);
  bool r = set != SET_RIGHT_TURN && a[RSIDE] > 0 && b[RSIDE] > 0 && fabsf(float(b[RSIDE] - a[RSIDE])) < 0.7f * fabsf(dF);
  float dL = float(b[LSIDE] - a[LSIDE]), dR = float(b[RSIDE] - a[RSIDE]);
  if (l && r && fabsf(dL + dR) > 40) {   // walls parallel: the two changes must cancel; else keep the smaller
    if (fabsf(dL) < fabsf(dR)) r = false;
    else l = false;
  }
  if (!l && !r) return false;
  float lat = l && r ? (dL - dR) / 2 : l ? dL : -dR;
  *deg = atanf(lat / dF) * 57.29578f;
  return true;
}

// The stage's 45 deg heading signal: at N the difference L45 - R45 (a sideways offset moves both the same
// way and mostly cancels), after a right turn L45, after a left turn R45. false: no echo or no reference.
static bool beam(const int16_t* ref, uint8_t set, const int16_t* us, float* v, float* r) {
  if (set == SET_ALL) {
    if (us[L45] <= 0 || us[R45] <= 0 || ref[L45] == NONE || ref[R45] == NONE) return false;
    *v = float(us[L45] - us[R45]);
    *r = float(ref[L45] - ref[R45]);
    return true;
  }
  uint8_t k = set == SET_RIGHT_TURN ? L45 : R45;
  if (us[k] <= 0 || ref[k] == NONE) return false;
  *v = us[k];
  *r = ref[k];
  return true;
}

// Heading from the last driveForward's fit: side offset against front reading over >= 6 front echoes
// spanning >= 50 mm; the slope is tan(heading) (see skewFrom; a fit over the whole drive instead of two
// readings, so the side walls' +-15 mm noise averages out).
bool Runner::driveSkew(float* deg) const {
  if (rN < 6 || rFmax - rFmin < 50) return false;
  float den = rN * rFF - rF * rF;
  if (den <= 0) return false;
  float slope = (rN * rFL - rF * rL) / den;
  if (fabsf(slope) > 0.7f) return false;
  *deg = atanf(slope) * 57.29578f;
  return true;
}

// After the colour turn the front must see the side wall at about the turned capture's reading; no echo
// or far off (2 Nov: 2.3 m, the robot then crept out of the field): turn 10 deg toward the 45 deg beam
// with the nearer echo, at most 3 times, before any creep.
bool Runner::faceExpected(int16_t want, int16_t us[NUS]) {
  for (uint8_t i = 0; i < 3 && (us[FRONT] <= 0 || fabsf(float(us[FRONT] - want)) > 250); ++i) {
    int8_t dir = us[L45] > 0 && (us[R45] <= 0 || us[L45] < us[R45]) ? 1 : us[R45] > 0 ? -1 : 0;
    if (!dir) break;
    io.event(RL_F("face_expected"), us[FRONT], want, dir);
    setGoal(goal + 10 * dir);
    TRY(rotateTo(goal, ROT_TOL, false));
    TRY(measure(us));
  }
  return true;
}

// Placed pointing roughly at the main wall: if the front beam has no echo but a 45 deg beam does, turn
// 15 deg toward that beam (at most 3 times) so the creep heads for the wall.
bool Runner::faceWall(int16_t us[NUS]) {
  for (uint8_t i = 0; i < 3 && us[FRONT] <= 0; ++i) {
    int8_t dir = us[L45] > 0 && (us[R45] <= 0 || us[L45] < us[R45]) ? 1 : us[R45] > 0 ? -1 : 0;
    if (!dir) break;
    io.event(RL_F("face_wall"), dir, us[L45], us[R45]);
    setGoal(goal + 15 * dir);
    TRY(rotateTo(goal, ROT_TOL, false));
    TRY(measure(us));
  }
  return true;
}

// Turn out the heading measured over the straight move from readings a to us (skewFrom); the gyro then
// holds the squared heading. Without a usable side wall the heading is kept.
bool Runner::straighten(uint8_t set, const int16_t a[NUS], int16_t us[NUS], bool* known, float maxTurn) {
  float th, y;
  if (known) *known = false;
  bool fit = driveSkew(&th);
  rN = 0;   // used once
  if (!fit && !skewFrom(set, a, us, &th)) { io.event(RL_F("skew_unknown"), a[FRONT], us[FRONT]); return true; }
  if (!io.yaw(&y)) return true;
  if (known) *known = true;
  io.event(RL_F("skew_src"), fit ? 1 : 0);
  th = clampf(th, -maxTurn, maxTurn);
  io.event(RL_F("skew"), i16(th * 10), us[FRONT] - a[FRONT]);
  if (fabsf(th) <= 1.5f) { setGoal(y); return true; }
  setGoal(y - th);
  int16_t was[NUS];
  memcpy(was, us, sizeof(was));
  TRY(rotateTo(goal, ROT_TOL, false));
  TRY(measure(us));
  // the turn also shows how the 45 deg beams answer to heading: mm per deg, for align's fine heading
  float y1, b0, b1, r;
  if (fabsf(th) >= 2 && io.yaw(&y1) && fabsf(y1 - y) >= 1.5f && beam(cal.ref[N], set, was, &b0, &r) && beam(cal.ref[N], set, us, &b1, &r)) {
    float sl = (b1 - b0) / (y1 - y);
    if (fabsf(sl) >= 3 && fabsf(sl) <= 40) beamSlope[set] = beamSlope[set] ? 0.5f * (beamSlope[set] + sl) : sl;
    io.event(RL_F("beam_slope"), set, i16(sl * 10), i16(beamSlope[set] * 10));
  }
  return true;
}

// Heading check on a pose: a straight forward move of `dist` (front reading) and the side wall(s) of the
// set give the heading (skewFrom), which is turned out; the caller aligns again afterwards.
bool Runner::squareByMove(uint8_t set, float dist, int16_t us[NUS]) {
  if (us[FRONT] <= 0) return true;
  int16_t a[NUS];
  memcpy(a, us, sizeof(a));
  TRY(driveForward(us[FRONT] - dist, DRIVE_MAX_MS, DRIVE_PWM, us));
  return straighten(set, a, us);
}

// After a wall stop (10 cm corner stop or 2 cm rule): localize again instead of going on blind. Back
// off 80 mm straight (the one direction with no wall), the side wall(s) of the set giving the heading
// over that move (skewFrom), turned out; the caller then goes on to its target from the fresh readings.
bool Runner::recover(uint8_t set, int16_t us[NUS], float maxTurn) {
  cornered = false;
  io.event(RL_F("recover"), us[FRONT], us[L45], us[R45]);
  if (us[FRONT] <= 0) {   // no front echo to track a drive: one timed reverse
    TRY(pulse(-100, 0, wcomp[0][0], PWM_X, back.msFor(80)));
    return measure(us);
  }
  int16_t a[NUS];
  memcpy(a, us, sizeof(a));
  TRY(driveForward(us[FRONT] + 80, DRIVE_MAX_MS, DRIVE_PWM, us));
  cornered = false;
  return straighten(set, a, us, 0, maxTurn);
}

// One continuous omnidirectional move onto a calibrated pose: forward and sideways at the same time,
// the wheel mix re-aimed on every poll at what is left of the error (sideways scaled up by STRAFE_EFF),
// so the robot slides along a straight line to the target instead of axis by axis. The front, side
// and 45 deg readings are each tracked by a Kalman filter while moving (FOCUS_ALL); an axis is done
// once its predicted error after braking (error + rate * stop time, learned) is inside tol or has
// crossed zero, and the robot stops when both are. Speed falls from the full creep / strafe power 200 mm
// out to just above breakaway, rising 4 PWM per 400 ms without progress. It also stops after
// MOVE_MAX_MS, when a tracked wall is lost, or when a front corner closes under cornerMm.
bool Runner::driveTo(Pose pose, uint8_t set, float tolX, float tolY, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  float d[NUS], p[NUS];
  toF(us, d);
  float ex0, ey0, ex, ey, exP, eyP;
  bool hx, hy, h1, h2;
  posErr(ref, set, d, &ex0, &hx, &ey0, &hy);
  bool doneX = !hx || fabsf(ex0) <= tolX, doneY = !hy || fabsf(ey0) <= tolY;
  // Undershoot by design: an axis stops at 30 % of its starting error (2 Nov field: aiming at zero, the
  // filters' lag overshot strafes by 2x); the next move takes the rest, so the error shrinks each move.
  const float stopX = fabsf(ex0) * 0.3f > tolX ? fabsf(ex0) * 0.3f : tolX, stopY = fabsf(ey0) * 0.3f > tolY ? fabsf(ey0) * 0.3f : tolY;
  if (doneX && doneY) return true;
  Kalman1D kf[NUS];
  bool live[NUS];
  uint8_t blind[NUS];
  for (uint8_t k = 0; k < NUS; ++k) {
    live[k] = us[k] > 0;
    if (live[k]) kf[k].init(float(us[k]));
    blind[k] = 0;
  }
  float yaw0, yaw1;
  bool hasYaw = io.yaw(&yaw0);
  ex = exP = ex0;
  ey = eyP = ey0;
  float t = 0, tWin = 0, nWin = fabsf(ex0) + fabsf(ey0), rx = 0, ry = 0;
  int8_t mx = 0, my = 0, mw = 0;
  uint8_t pwm = 0, boost = 0, end = 0;   // end: 0 reached, 1 time, 2 wall lost, 3 corner, 4 travelled too far
  // A move never runs much past its error (2 Nov at N: a 16 mm correction drove 120 mm when the front's
  // two echoes made the filter reject it): time scaled to the error, and the raw front's travel capped.
  const float big = fabsf(ex0) > fabsf(ey0) / STRAFE_EFF ? fabsf(ex0) : fabsf(ey0) / STRAFE_EFF;
  const float capMs = clampf(500 + 10 * big, 600, MOVE_MAX_MS);
  const int16_t f0 = us[FRONT];
  bool moving = false;
  for (uint16_t polls = 0;; ++polls) {
    float cx = doneX ? 0 : ex, cy = doneY ? 0 : ey / STRAFE_EFF;
    float m = fabsf(cx) > fabsf(cy) ? fabsf(cx) : fabsf(cy);
    if (m < 0.5f) break;
    int8_t nx = int8_t(roundf(100 * cx / m)), ny = int8_t(roundf(100 * cy / m));
    if (nx > -8 && nx < 8) nx = 0;
    if (ny > -8 && ny < 8) ny = 0;
    if (towardWall(nx, 0)) nx = 0;   // 2 cm rule: that direction is closed
    if (towardWall(0, ny)) ny = 0;
    if (!nx && !ny) { io.event(RL_F("wall_block"), i16(cx), i16(cy), blocked); end = 3; break; }
    float ax = fabsf(nx), ay = fabsf(ny);
    float top = (ax * DRIVE_PWM + ay * SIDE_PWM) / (ax + ay), lo = (ax * CREEP_FLOOR + ay * (SIDE_PWM - 6)) / (ax + ay);
    float left = fabsf(doneX ? 0 : exP) > fabsf(doneY ? 0 : eyP) ? fabsf(exP) : fabsf(eyP);
    uint8_t np = uint8_t(clampf(lo + (top - lo) * clampf(left / 200, 0, 1) + boost, 30, 120) + 0.5f);
    int8_t nw = int8_t(roundf((ax * (nx ? wcomp[0][nx > 0] : 0) + ay * (ny ? wcomp[1][ny > 0] : 0)) / (ax + ay)));
    if (!moving) {
      if (!io.driveStart(nx, ny, nw, np, set == SET_ALL ? FOCUS_ALL : set == SET_RIGHT_TURN ? FOCUS_LEFT : FOCUS_RIGHT)) return fail(RL_F("stopped"));
      moving = true;
      mx = nx; my = ny; mw = nw; pwm = np;
    } else if (fabsf(float(nx - mx)) > 4 || fabsf(float(ny - my)) > 4 || nw != mw || np + 2 <= pwm || np >= pwm + 2) {
      if (!io.driveSet(nx, ny, nw, np)) { io.driveEnd(); return fail(RL_F("stopped")); }
      mx = nx; my = ny; mw = nw; pwm = np;
    }
    int16_t z[NUS];
    float dt;
    if (!io.drivePoll(z, &dt)) { io.driveEnd(); return fail(RL_F("stopped")); }
    t += dt * 1000;
    noteWalls(z);   // a newly closed direction is dropped from the mix on the next aim
    for (uint8_t k = 0; k < NUS; ++k) {
      if (live[k]) kf[k].predict(dt);
      if (z[k] == NOT_PINGED) continue;
      if (z[k] > 0) {
        if (!live[k] || (k != FRONT && blind[k] >= 3)) { kf[k].init(float(z[k])); live[k] = true; blind[k] = 0; }
        else if (kf[k].update(float(z[k]))) blind[k] = 0;
        else ++blind[k];
      } else ++blind[k];
    }
    for (uint8_t k = 0; k < NUS; ++k) {
      d[k] = live[k] && blind[k] < 3 ? kf[k].dist() : -1;
      p[k] = d[k] > 0 ? d[k] + kf[k].speed() * (k == FRONT ? stopTau : stopTauY) : -1;
    }
    posErr(ref, set, d, &ex, &h1, &ey, &h2);
    if ((hx && !doneX && !h1) || (hy && !doneY && !h2)) { end = 2; break; }
    posErr(ref, set, p, &exP, &h1, &eyP, &h2);
    rx = live[FRONT] ? kf[FRONT].speed() : 0;
    ry = stopTauY > 0 ? (eyP - ey) / stopTauY : 0;
    // not in the first 150 ms: the speed estimate is still noise (2 Nov: moves ended after 55 ms, unmoved)
    if (t >= 150 && !doneX && (fabsf(exP) <= stopX || exP * ex0 < 0)) doneX = true;
    if (t >= 150 && !doneY && (fabsf(eyP) <= stopY || eyP * ey0 < 0)) doneY = true;
    if (polls % 8 == 1) io.event(RL_F("mpoll"), i16(t), i16(ex), i16(ey), pwm);
    if (doneX && doneY) break;
    bool nearWall = !live[FRONT] || kf[FRONT].dist() < CORNER_ZONE;   // see driveForward
    float cornerCap = live[FRONT] && !(PARK_ORIG && inPark) ? kf[FRONT].dist() - CORNER_MARGIN : 1e6f;   // the filter carries over a missed echo or two
    if (mx > 0 && ((nearWall && ((live[L45] && blind[L45] < 2 && kf[L45].dist() < cornerMm && kf[L45].dist() < cornerCap) ||
                                 (live[R45] && blind[R45] < 2 && kf[R45].dist() < cornerMm && kf[R45].dist() < cornerCap))) ||
                   (z[FRONT] > 0 && z[FRONT] < 40) ||   // the filtered front: a single short echo is not a wall (2 Nov at N)
                   (d[FRONT] > 0 && d[FRONT] < (ref[FRONT] != NONE && ref[FRONT] - 15 > 40 ? ref[FRONT] - 15 : 40)))) {
      io.event(RL_F("corner_stop"), i16(kf[L45].dist()), i16(kf[R45].dist()));
      end = 3;
      break;
    }
    if (t >= capMs) { end = 1; break; }
    if (f0 > 0 && z[FRONT] > 0 && fabsf(float(f0 - z[FRONT])) > fabsf(ex0) + 50) { end = 4; break; }
    if (t - tWin >= 400) {   // no progress at all over 400 ms: a little more power
      if (nWin - (fabsf(ex) + fabsf(ey)) < 3 && boost < 30) boost += 4;
      tWin = t;
      nWin = fabsf(ex) + fabsf(ey);
    }
  }
  float exD = ex, eyD = ey;
  cornered = end == 3;
  if (moving && !io.driveEnd()) return fail(RL_F("stopped"));
  TRY(measure(us));
  toF(us, d);
  float ex1, ey1;
  posErr(ref, set, d, &ex1, &h1, &ey1, &h2);
  // stopping times from where the robot really stopped (as driveForward / driveSide)
  if (h1 && fabsf(rx) > 60) stopTau = clampf(0.5f * stopTau + 0.5f * (ex1 - exD) / rx, 0.01f, 0.8f);
  if (h2 && fabsf(ry) > 30) stopTauY = clampf(0.5f * stopTauY + 0.5f * (ey1 - eyD) / ry, 0.01f, 0.5f);
  // the move turned the robot: learn the rotation to mix in for its main axis
  if (hasYaw && t > 300 && io.yaw(&yaw1) && (mx || my)) {
    uint8_t a = fabsf(float(mx)) >= fabsf(float(my)) ? 0 : 1;
    bool s = (a ? my : mx) > 0;
    wcomp[a][s] = int8_t(clampf(roundf(wcomp[a][s] - (yaw1 - yaw0) * (a ? 1.0f : 1.5f) / (t > 600 ? 2.0f : 1.0f)), -60, 60));
  }
  io.event(RL_F("move"), i16(ex0), i16(ey0), h1 ? i16(ex1) : NONE, h2 ? i16(ey1) : NONE);
  io.event(RL_F("move_end"), end, i16(t), pwm, boost);
  return true;
}

// Feedback onto a calibrated pose: measure (once; then the readings each move ends with), placement
// accuracy over the stage's sensors (100 - rms mm, see accuracySet); done at `target`, when both
// measured axes are inside deadX / deadY mm, or after maxMoves; otherwise one combined move (driveTo) and the
// gyro heading held. An imperfect pose never ends the run: it goes on from there, logged.
bool Runner::align(Pose pose, uint8_t set, float target, float deadX, float deadY, uint8_t maxMoves, int16_t us[NUS]) {
  float best = 0;
  uint8_t beamFixes = 0, recoveries = 0;
  for (uint8_t move = 0;; ++move) {
    if (move == 0) TRY(measure(us));   // later rounds use the readings each move / heading hold ends with
    float d[NUS], ex, ey;
    bool hx, hy;
    toF(us, d);
    posErr(cal.ref[pose], set, d, &ex, &hx, &ey, &hy);
    float acc = accuracySet(pose, set, us);
    if (acc > best) best = acc;
    io.event(RL_F("align"), pose * 100 + move, hx ? i16(ex) : NONE, hy ? i16(ey) : NONE, i16(acc));
    // In position (front and side within 12 mm): the 45 deg beams against the capture give the heading
    // left over, with the slope learned from the straighten turns (at most 2 such turns, 5 deg each).
    float bv, br;
    if (beamFixes < 2 && hx && hy && fabsf(ex) <= 12 && fabsf(ey) <= 12 && fabsf(beamSlope[set]) >= 8 &&   // 2 Nov: -3 mm/deg gave a wrong 5 deg
        beam(cal.ref[pose], set, us, &bv, &br) && fabsf(bv - br) > 6) {
      float dth = clampf(-(bv - br) / beamSlope[set], -5, 5);
      if (fabsf(dth) > 1.5f) {
        ++beamFixes;
        io.event(RL_F("beam_fix"), pose, i16(bv - br), i16(dth * 10));
        setGoal(goal + dth);
        TRY(rotateTo(goal, ROT_TOL, false));
        TRY(measure(us));
        continue;
      }
    }
    bool inBand = hx && hy && fabsf(ex) <= deadX && fabsf(ey) <= deadY;
    if (acc >= target || inBand || move >= maxMoves || (!hx && !hy)) {
      io.event(RL_F("aligned"), pose, i16(acc), i16(best), acc >= target ? 1 : 0);
      return true;
    }
    TRY(driveTo(pose, set, 0.7f * deadX, 0.7f * deadY, us));
    if (cornered && recoveries < 2) { ++recoveries; TRY(recover(set, us)); continue; }
    TRY(hold(us));
  }
}

// How well the turned stage's three sensors match a capture: rms mm over front, the side on the main wall
// and the 45 deg beam on that side (each capped at 150, no echo = 150).
static float fitErr(const int16_t* ref, uint8_t set, const int16_t* us) {
  const uint8_t ks[3] = {FRONT, uint8_t(set == SET_RIGHT_TURN ? LSIDE : RSIDE), uint8_t(set == SET_RIGHT_TURN ? L45 : R45)};
  float sum = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    uint8_t k = ks[i];
    if (ref[k] == NONE) continue;
    float e = us[k] > 0 ? fabsf(float(us[k] - ref[k])) : 150;
    if (e > 150) e = 150;
    sum += e * e;
    ++n;
  }
  return n ? sqrtf(sum / n) : 0;
}

// After the gyro's 90 deg: the heading that best fits the turned capture on its three sensors (user,
// 2 Nov: the gyro turn alone overshot on the field). From the gyro's heading, 3 deg steps one way while
// the fit improves by > 3 mm (at most 3 steps), else the other way; the best heading becomes the goal.
bool Runner::fitHeading(Pose pose, uint8_t set, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[pose];
  TRY(measure(us));
  float y0, y;
  if (!io.yaw(&y0)) return true;
  float best = fitErr(ref, set, us), bestYaw = y0;
  io.event(RL_F("fit_heading"), 0, i16(best), 0);
  for (int8_t dir = 1; dir >= -1; dir -= 2) {
    bool improved = false;
    for (uint8_t step = 1; step <= 3; ++step) {
      TRY(rotateTo(bestYaw + 3 * dir, ROT_TOL, false));
      TRY(measure(us));
      if (!io.yaw(&y)) break;
      float e = fitErr(ref, set, us);
      io.event(RL_F("fit_heading"), dir * step, i16(e), i16((y - y0) * 10));
      if (e < best - 3) { best = e; bestYaw = y; improved = true; }
      else break;
    }
    if (improved) break;
    TRY(rotateTo(bestYaw, ROT_TOL, false));   // back to the best before trying the other way
  }
  setGoal(bestYaw);
  TRY(rotateTo(goal, ROT_TOL, false));
  io.event(RL_F("fit_heading_done"), i16(best), i16((goal - y0) * 10));
  return measure(us);
}

// Into the box (2 Nov field: sideways moves at the wall each turned the robot 3-7 deg, the heading fix
// then moved it sideways again and it went in and out of the spot). Sideways is settled out in the open,
// then one straight creep goes in, and only small single-axis fixes are made at the wall:
//  1 from ~180 mm short: continuous strafes onto the box's line (side sensor on the main wall), heading
//    held between them, until within 5 mm (3 strafes at most)
//  2 one slow straight creep onto the box's front reading; the side wall fitted over it gives the heading,
//    turned out (at most 3 deg this close to the wall)
//  3 fixes, one axis at a time: front by a straight drive if off by > 5 mm, sideways by a strafe only if
//    off by > 8 mm, heading held (<= 3 deg near the wall); 4 rounds at most
bool Runner::parkIn(Pose box, uint8_t set, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[box];
  float d[NUS], ex, ey;
  bool hx, hy, used;
  for (uint8_t i = 0; i < 3; ++i) {                                   // 1
    toF(us, d);
    posErr(ref, set, d, &ex, &hx, &ey, &hy);
    io.event(RL_F("park_line"), i, hy ? i16(ey) : NONE, us[FRONT]);
    if (!hy || fabsf(ey) <= (PARK_ORIG ? 5 : PARK_DEAD_Y)) break;   // 3 Nov sim: at 5 mm a 6 mm error strafed +6 -20 +10 (overshoot)
    TRY(driveSide(box, 0.9f, us, &used));
    if (!used) break;
    TRY(hold(us));
  }
  int16_t a[NUS];                                                     // 2
  memcpy(a, us, sizeof(a));
  if (us[FRONT] > 0 && us[FRONT] - ref[FRONT] > 15) {
    skewMode = 1 + set;
    TRY(driveForward(ref[FRONT], CREEP_MAX_MS, DRIVE_PWM, us));
    skewMode = 0;
    // at the wall an 80 mm back-off is too short for a big heading fix (3 Nov sim: a noisy 9.5 deg parked 6 deg off)
    if (cornered) TRY(recover(set, us, PARK_ORIG ? 35 : PARK_RECOVER_MAX));
    else TRY(straighten(set, a, us, 0, PARK_FINAL_SKEW_MAX));
  }
  for (uint8_t i = 0; i < 4; ++i) {                                   // 3
    TRY(measure(us));
    toF(us, d);
    posErr(ref, set, d, &ex, &hx, &ey, &hy);
    io.event(RL_F("park_fix"), i, hx ? i16(ex) : NONE, hy ? i16(ey) : NONE);
    bool fixX = hx && fabsf(ex) > 5, fixY = hy && fabsf(ey) > (PARK_ORIG ? 8 : PARK_DEAD_Y);
    if (!fixX && !fixY) break;
    if (fixX) TRY(driveForward(ref[FRONT], DRIVE_MAX_MS, DRIVE_PWM, us));
    else TRY(driveSide(box, 0.8f, us, &used));
    TRY(hold(us));
  }
  return true;
}

// Last adjust in the box, time-boxed (user, 3 Nov): while time is left, fix the errors the robot can
// see standing still and end as soon as there are none.
//  - front off the box's reading by > FINAL_TOL_X: one straight drive onto it
//  - sideways off by > FINAL_TOL_Y: back off FINAL_BACK (0: none) so the strafe is made away from the
//    side wall, strafe onto the box's line, heading held by the gyro, straight back in
// The heading is not seen standing still (one sensor on the main wall), and a read from a straight move
// carries the robot's crab (3 Nov sim: the same -6 deg out and back on a square robot; sd 2.5 deg), so a
// move's read turns the robot only if the way out and the way back agree and the mean is off by more
// than FINAL_SKEW_MIN (at most FINAL_MAX_TURN). FINAL_HEAD > 0.5 also makes that out-and-back check once
// when nothing else is seen. A fix only starts if FINAL_CYCLE_MS of the budget is left.
// 3 Nov sim tune (300 generations, error + time): front within 4 mm, sideways left to parkIn's dead band,
// no back-off, no heading check (both cost more than they fixed). FINAL_TOL_Y 10: along-wall 6.9 -> 5.6 mm
// for ~1 s and 0.4 more strafes.
TUNE(float) FINAL_MS = 13000;         // ms: time budget (0 = off)
TUNE(float) FINAL_CYCLE_MS = 3600;    // ms: what one strafe fix takes, about
TUNE(float) FINAL_TOL_X = 4;          // mm
TUNE(float) FINAL_TOL_Y = 17;         // mm
TUNE(float) FINAL_BACK = 0;           // mm
TUNE(float) FINAL_SKEW_MIN = 3;       // deg
TUNE(float) FINAL_MAX_TURN = 4;       // deg
TUNE(float) FINAL_HEAD = 0;           // > 0.5: one heading check even when nothing else is seen
// The fixes start gently (user, 3 Nov: the wheels' soft start was too hard for these small moves and they
// overshot): the ramp-up is lowered to FINAL_RAMP PWM per 10 ms for the whole adjust, then restored.
TUNE(float) FINAL_RAMP = 8;           // PWM per 10 ms while speeding up (default 8: 0 -> 68 in 85 ms; here 340 ms)
bool Runner::finalAdjust(Pose box, uint8_t set, int16_t us[NUS]) {
  if (FINAL_MS <= 0) return true;
  io.setRamp(uint8_t(FINAL_RAMP + 0.5f));
  bool ok = finalAdjustMoves(box, set, us);
  io.setRamp(8);
  return ok;
}

bool Runner::finalAdjustMoves(Pose box, uint8_t set, int16_t us[NUS]) {
  const int16_t* ref = cal.ref[box];
  const uint32_t t0 = io.ms();
  float d[NUS], ex, ey, th1, th2, y0, y1, y2, y3, yHome;
  bool hx, hy, used, headDone = FINAL_HEAD <= 0.5f;
  if (!io.yaw(&yHome)) return true;   // the heading parkIn left: every fix comes back to it (gyro, short moves)
  for (uint8_t cyc = 0; cyc < 6; ++cyc) {
    const float left = FINAL_MS - float(io.ms() - t0);
    TRY(measure(us));
    toF(us, d);
    posErr(ref, set, d, &ex, &hx, &ey, &hy);
    bool fixX = hx && fabsf(ex) > FINAL_TOL_X, fixY = hy && fabsf(ey) > FINAL_TOL_Y;
    io.event(RL_F("final_see"), cyc, hx ? i16(ex) : NONE, hy ? i16(ey) : NONE, i16(left / 100));
    if (!fixX && !fixY && headDone) { io.event(RL_F("final_ok"), cyc); break; }
    if (!fixY && headDone) {                                            // front only: short and straight
      if (left < 0.4f * FINAL_CYCLE_MS) { io.event(RL_F("final_time"), cyc); break; }
      TRY(driveForward(ref[FRONT], DRIVE_MAX_MS, DRIVE_PWM, us));
      cornered = false;
      TRY(home(yHome));
      continue;
    }
    if (left < FINAL_CYCLE_MS || us[FRONT] <= 0 || !io.yaw(&y0)) { io.event(RL_F("final_time"), cyc); break; }
    headDone = true;
    int16_t a[NUS];
    bool k1 = false, k2 = false;
    if (FINAL_BACK >= 30) {
      memcpy(a, us, sizeof(a));
      skewMode = 1 + set;
      TRY(driveForward(us[FRONT] + FINAL_BACK, DRIVE_MAX_MS, DRIVE_PWM, us));
      skewMode = 0;
      cornered = false;
      k1 = driveSkew(&th1) || skewFrom(set, a, us, &th1);
      rN = 0;
      TRY(measure(us));
      if (!io.yaw(&y1)) break;
      if (k1) th1 += 0.5f * (y1 - y0);   // the read is the mean over the move: carried to its end
    } else y1 = y0;
    if (fixY) {
      TRY(driveSide(box, 0.9f, us, &used));
      TRY(hold(us));
      TRY(measure(us));
    }
    if (us[FRONT] <= 0 || !io.yaw(&y2)) break;
    memcpy(a, us, sizeof(a));
    if (us[FRONT] - ref[FRONT] > 30) {
      skewMode = 1 + set;
      TRY(driveForward(ref[FRONT], DRIVE_MAX_MS, DRIVE_PWM, us));
      skewMode = 0;
      cornered = false;
      k2 = driveSkew(&th2) || skewFrom(set, a, us, &th2);
      rN = 0;
      TRY(measure(us));
    }
    if (!io.yaw(&y3)) break;
    if (k2) th2 += 0.5f * (y3 - y2);
    if (k1) th1 += y3 - y1;            // the first read carried to now by the gyro (short moves: kept)
    float th = k1 && k2 ? 0.5f * (th1 + th2) : 0;
    io.event(RL_F("final_head"), k1 ? i16(th1 * 10) : NONE, k2 ? i16(th2 * 10) : NONE, i16(th * 10));
    if (k1 && k2 && th1 * th2 > 0 && fabsf(th) > FINAL_SKEW_MIN) {
      setGoal(y3 - clampf(th, -FINAL_MAX_TURN, FINAL_MAX_TURN));
      TRY(rotateTo(goal, ROT_TOL, false));
      yHome = goal;
    } else TRY(home(yHome));
  }
  return true;
}

// Back onto heading y after a short fix move (the gyro kept it), if the move turned the robot > 1 deg.
bool Runner::home(float y) {
  float now;
  setGoal(y);
  if (!io.yaw(&now) || fabsf(now - y) <= 1.0f) return true;
  return rotateTo(y, ROT_TOL, false);
}

// Steep starts (3 Nov field, IMG_3994: placed on the arc ~40 deg out, pointing at N, the creep to a 50 cm
// front reading overshot N along the slanted beam and the run ended crooked at the right wall). The robot
// is placed pointing at N, so N lies straight ahead; only the angle to the main wall is unknown.
//  1 the front at the start heading (f0), then one 12 deg probe turn (gyro) toward the wall's normal
//    (the reading falls); if it grows, that was away from it: probe 12 deg the other side instead
//  2 the two readings fix the angle: a flat wall reads (perpendicular distance) / cos(angle off the
//    normal - the beam's half-width), the front sitting FRONT_FWD ahead of the turning centre
//  3 back to the start heading and one straight creep at it (heading held between <= 2 s legs, the
//    gyro FIFO) until the front reads f0 x (N's distance / the start's), both measured along the same
//    slanted beam: the ratio holds whatever the beam width, so the creep stops on N's line
//  4 turn square to the wall (the start heading + the angle); step 2 then settles onto N as before
// A small angle (no clear slope) or anything unexpected: *done = false, the start is left as it was.
TUNE(float) START_PROBE = 12;     // deg
TUNE(float) START_CONE = 11;      // deg: nominal beam half-width for the angle (its error only offsets
                                         // the final turn; step 2 squares on the side walls)
TUNE(float) START_MIN_DEG = 12;   // below this the usual creep copes (sim: starts up to ~30 deg)
TUNE(float) START_LEG_MS = 2300;  // one leg of the walk: + soft start and brake stays inside the gyro FIFO (2.5 s)
TUNE(uint8_t) START_PWM = 52;
TUNE(float) START_DROP = 0.02f;         // a probe reading this share lower: the normal is that way
TUNE(float) START_ASYM = 30;            // mm: else the two probes' difference decides the side
TUNE(float) START_FIT_MAX = 25;         // mm: the angle solve's misfit above which the start is left alone
TUNE(float) START_MIN_TRAVEL = 100;     // mm: a walk shorter than this is left to the usual creep    // walk top speed on the open floor; the creep profile slows it to 41 over the last 25 cm
static float startModel(float alpha, float dc, float b) {   // front reading at b deg off the normal
  float off = b - START_CONE;
  return (dc - FRONT_FWD * cosf(b * DEG)) / cosf((off > 0 ? off : 0) * DEG);
}
// Steep start: find the main wall's normal by turning toward it in START_STEP steps (gyro), the front read
// at each: its shortest reading is the wall straight ahead (flat over the beam's width: the middle of the
// readings within 15 mm of the shortest). That gives the angle a of the start heading off square and the
// distance to the wall. N lies straight along the start heading (the robot is placed pointing at it), so
// it walks back along it; the front is no use there (the far corner), but the 45 deg beam on the wall's
// side is within ~20 deg of square to the main wall at these angles: it reads the wall's perpendicular
// distance, and the walk stops where that puts the robot on N's line. Then it squares to the wall.
TUNE(float) START_STEP = 10;          // deg per search step
TUNE(float) START_SEARCH_MAX = 85;    // deg: furthest the search turns from the start heading
TUNE(float) STEEP_RATIO = 0.6f;       // the shorter 45 deg reading / the front's below this: a steep start
TUNE(float) STEEP_NEAR_MAX = 1300;    // mm: ... and that 45 deg reading is a wall this close
static const float SIDE45_LAT = 120;  // mm: the 45 deg sensors sit this far left / right of the centre line
bool Runner::steepStart(float y0, int8_t dir, int16_t us[NUS], bool* done) {
  // the main wall straight ahead reads far shorter than the far corner the start heading saw (sim: 680-800
  // against 1700-1970): a longer reading on the way is still the corner, and is never the minimum
  const float fCap = us[FRONT] > 0 ? 0.8f * us[FRONT] : 1400;
  // ... and not much nearer than the main wall the 45 deg beam on its side reads now (sim: from beside a
  // side wall's end the search's front met that wall's back face at 313 mm, the 45 deg beam read 618)
  const int16_t s45 = us[dir > 0 ? L45 : R45];
  const float fFloor = s45 > 0 ? s45 - 150 : 0;
  float yaws[10];
  int16_t fr[10];
  uint8_t n = 0, rise = 0;
  float fBest = 1e9f, y;
  for (uint8_t step = 1; step < 10 && step * START_STEP <= START_SEARCH_MAX; ++step) {
    TRY(rotateTo(y0 + dir * step * START_STEP, ROT_TOL, false));
    TRY(measure(us));
    if (us[FRONT] <= 0) TRY(measure(us));
    if (!io.yaw(&y)) break;
    yaws[n] = y;
    fr[n++] = us[FRONT];
    io.event(RL_F("steep_step"), step, us[FRONT], us[dir > 0 ? L45 : R45], i16((y - y0) * 10));
    if (us[FRONT] > 0 && us[FRONT] < fCap && us[FRONT] > fFloor && us[FRONT] < fBest) { fBest = us[FRONT]; rise = 0; }
    // past the normal: two real readings clearly longer (a missing echo is not one: sim, near square the front
    // missed twice after the first wall reading and the search stopped 47 deg short)
    else if (fBest < 1e9f && us[FRONT] > fBest + (0.04f * fBest > 25 ? 0.04f * fBest : 25) && ++rise >= 2) break;
  }
  if (fBest >= 1e9f) {
    io.event(RL_F("steep_no_wall"));
    setGoal(y0);
    return rotateTo(goal, ROT_TOL, false);
  }
  float ys = 0;
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; ++i) if (fr[i] > fFloor && fr[i] <= fBest + 15) { ys += yaws[i]; ++m; }
  const float yN = ys / m, a = fabsf(yN - y0), dc = fBest + FRONT_FWD;
  const int16_t* nr = cal.ref[N];
  const float ca = cosf(a * DEG), sa = sinf(a * DEG), off = FRONT_FWD * ca + SIDE45_LAT * sa, dNc = nr[FRONT] + FRONT_FWD;
  const uint8_t k45 = dir > 0 ? L45 : R45;
  io.event(RL_F("steep_normal"), i16(a * 10), i16(fBest), m);
  bool walk = a >= START_MIN_DEG && dc - dNc >= START_MIN_TRAVEL && a <= 80;
  if (walk) {
    setGoal(y0);
    TRY(rotateTo(goal, ROT_TOL, false));
    TRY(measure(us));
    for (uint8_t t = 0; t < 4 && (us[k45] <= 0 || fabsf(us[k45] - (dc - off)) >= 150); ++t) TRY(measure(us));   // a stray echo voids it
    const float want = dNc - off, expect = dc - off;
    io.event(RL_F("steep_walk"), us[k45], i16(expect), i16(want), k45);
    if (us[k45] > 0 && fabsf(us[k45] - expect) < 150) TRY(walk45(a, dir, us));
    else io.event(RL_F("steep_beam_off"), us[k45], i16(expect));
  }
  setGoal(yN);   // square to the main wall
  TRY(rotateTo(goal, ROT_TOL, false));
  TRY(measure(us));
  io.event(RL_F("start_square"), us[FRONT], us[LSIDE], us[RSIDE]);
  *done = true;
  return true;
}

// Walk on along the heading (a deg off square, the main wall's normal on side dir) to N's line, stopped by
// the 45 deg beam on the wall's side: within ~|a - 45| of square it reads the sensor's perpendicular
// distance from the main wall, and the walk ends where that puts the centre at N's distance.
bool Runner::walk45(float a, int8_t dir, int16_t us[NUS]) {
  const uint8_t k45 = dir > 0 ? L45 : R45;
  const float ca = cosf(a * DEG), sa = sinf(a * DEG);
  const float want = cal.ref[N][FRONT] + FRONT_FWD - (FRONT_FWD * ca + SIDE45_LAT * sa);
  io.event(RL_F("walk45"), us[k45], i16(want), i16(a * 10), k45);
  stopK = k45;
  for (uint8_t leg = 0; leg < 12; ++leg) {
    for (uint8_t t = 0; t < 3 && us[k45] <= 0; ++t) TRY(measure(us));
    if (us[k45] <= 0 || us[k45] - want <= 15) break;
    TRY(driveForward(want, START_LEG_MS, START_PWM, us));
    if (cornered) { stopK = FRONT; TRY(recover(SET_ALL, us)); break; }
    TRY(hold(us));
  }
  stopK = FRONT;
  return true;
}

// Walk into the field first, no turning at the start (user, 3 Nov: on the start semicircle the real sensors
// may see nothing; the turn checks belong closer to the walls). The robot is placed pointing at N, so walking
// straight ahead keeps it on the line to N whatever the start angle; N is ~0.8 m+ ahead from any start, and
// START_NUDGE stays well short of it. Every later step (the angle probes, the steep search, the walk to N's
// line) only uses where the robot is on that line, so it works from here as from the start.
// It needs no echo: it ends after START_NUDGE mm tracked by the front (any surface: a far corner's reading also
// shrinks 1:1 along the heading), else after START_NUDGE_LEGS legs; and at once if anything reads close.
// Legs of <= START_NUDGE_LEG_MS (the gyro FIFO: 2.5 s), the heading held by the gyro between them.
// Slow and smooth (user, 3 Nov): gentle ramp-up (START_NUDGE_RAMP PWM per 10 ms instead of 8), eased down
// to START_NUDGE_END over the last 40 % of each leg.
TUNE(float) START_NUDGE = 0;        // mm (0 = off; user 3 Nov: twice the first 400)
TUNE(float) START_NUDGE_LEG_MS = 2200;
TUNE(float) START_NUDGE_LEGS = 4;     // legs when the front tracks nothing
TUNE(uint8_t) START_NUDGE_PWM = 47;   // open floor
TUNE(uint8_t) START_NUDGE_END = 41;
TUNE(float) START_NUDGE_RAMP = 2;
TUNE(float) START_NUDGE_NEAR = 450;   // mm: a front echo this close ends it (a wall ahead: the turn checks start here)
TUNE(float) START_NUDGE_NEAR45 = 250; // mm: a 45 deg echo this close ends it
// A start already seen as steep (a 45 deg beam reads the main wall: the steep check below) walks only this far:
// it needs no walk to see, and 400 mm along a steep line brings it beside a side wall's end, where the turning
// search then swept a corner into it (sim, +60 deg starts)
TUNE(float) START_NUDGE_STEEP = 150;   // (300 with the 800 mm walk: random-field wall touches 4.7 -> 7.2 %)
bool Runner::startNudge(int16_t us[NUS]) {
  memcpy(pre, us, sizeof(pre));
  if (START_NUDGE <= 0 || START_NUDGE_LEG_MS <= 0) return true;
  float goalMm = START_NUDGE;
  {
    const int16_t l = us[L45], r = us[R45], s45 = l > 0 && (r <= 0 || l < r) ? l : r;
    if (s45 > 0 && s45 < STEEP_NEAR_MAX && (us[FRONT] <= 0 || s45 < STEEP_RATIO * us[FRONT]) && START_NUDGE_STEEP < goalMm) goalMm = START_NUDGE_STEEP;
  }
  io.event(RL_F("start_nudge"), us[FRONT], us[L45], us[R45], i16(goalMm));
  float total = 0, tAll = 0;
  bool near = false;
  io.setRamp(uint8_t(START_NUDGE_RAMP + 0.5f));
  for (uint8_t leg = 0; leg < 8 && !near && total < goalMm; ++leg) {
    Kalman1D kf;
    bool live = us[FRONT] > 0;
    if (live) kf.init(float(us[FRONT]));
    const float f0 = live ? float(us[FRONT]) : 0;
    if (!live && leg >= START_NUDGE_LEGS) break;   // nothing tracked: the time-limited legs are done
    // this leg's share: the rest of the distance, but at most one leg's time
    const float legMm = goalMm - total;
    uint8_t pwm = START_NUDGE_PWM;
    if (!io.driveStart(100, 0, wcomp[0][1], pwm, FOCUS_FRONT)) { io.setRamp(8); return fail(RL_F("stopped")); }
    float t = 0, moved = 0;
    uint8_t closeF = 0, close45 = 0;   // close echoes in a row (one stray echo must not end the walk)
    for (;;) {
      int16_t z[NUS];
      float dt;
      if (!io.drivePoll(z, &dt)) { io.driveEnd(); io.setRamp(8); return fail(RL_F("stopped")); }
      t += dt * 1000;
      if (live) kf.predict(dt);
      if (z[FRONT] > 0 && z[FRONT] != NOT_PINGED) {
        if (!live) { kf.init(float(z[FRONT])); live = true; }
        else kf.update(float(z[FRONT]));
        closeF = z[FRONT] < START_NUDGE_NEAR ? closeF + 1 : 0;
        if (closeF >= 2) { near = true; break; }
      }
      for (uint8_t k = L45; k <= R45; ++k) if (z[k] != NOT_PINGED) close45 = z[k] > 0 && z[k] < START_NUDGE_NEAR45 ? close45 + 1 : 0;
      if (close45 >= 2) { near = true; break; }
      if (live && f0 > 0) moved = f0 - kf.dist();
      // ease down over the last 40 % of the leg (distance when tracked, else time)
      const float done = live && f0 > 0 ? moved / legMm : 0;
      const float doneT = t / START_NUDGE_LEG_MS;
      const uint8_t want = (done > 0.6f || doneT > 0.6f) ? START_NUDGE_END : START_NUDGE_PWM;
      if (want != pwm) { pwm = want; if (!io.driveSet(100, 0, wcomp[0][1], pwm)) { io.driveEnd(); io.setRamp(8); return fail(RL_F("stopped")); } }
      if (live && f0 > 0 && moved >= legMm) break;
      if (t >= START_NUDGE_LEG_MS) break;
    }
    if (!io.driveEnd()) { io.setRamp(8); return fail(RL_F("stopped")); }
    TRY(measure(us));
    total += moved;
    tAll += t;
    io.event(RL_F("start_nudged"), leg, i16(t), i16(moved), us[FRONT]);
    io.setRamp(8);
    TRY(hold(us));   // back onto the start heading (each leg inside the gyro FIFO)
    io.setRamp(uint8_t(START_NUDGE_RAMP + 0.5f));
  }
  io.setRamp(8);
  io.event(RL_F("start_walked"), i16(total), i16(tAll / 100), near);
  return true;
}

// First thing in the run (user, 3 Nov evening): straight forward START_FWD_MM along the start heading,
// measured by the wheel encoders (no echo needed), then the run as before (Reinforced + adjust).
// ENC_TICKS_PER_MM: all four wheels summed, channel A rising edges. Not measured on this robot yet: the kit's
// 11-pulse encoder x 1:30 gearbox = 330 per wheel turn, 80 mm wheel = 251 mm -> 1.31 per wheel, 5.25 for
// four (the turns of the 3 Nov run, 18.9 ticks per deg, fit that). The start_fwd line logs ticks, time and
// the front before / after, to check it on the field. Legs of <= START_FWD_LEG_MS (the gyro FIFO, 2.5 s),
// the heading held by the gyro between them; it stops early if anything echoes within START_FWD_NEAR ahead.
TUNE(float) START_FWD_MM = 0;       // mm (0 = off)
TUNE(float) ENC_TICKS_PER_MM = 5.25f; // four wheels summed
TUNE(uint8_t) START_FWD_PWM = 43;     // the creep's top speed
TUNE(float) START_FWD_LEG_MS = 2200;
TUNE(float) START_FWD_NEAR = 300;     // mm
bool Runner::startForward(int16_t us[NUS]) {
  if (START_FWD_MM <= 0 || ENC_TICKS_PER_MM <= 0) return true;
  const uint32_t want = uint32_t(START_FWD_MM * ENC_TICKS_PER_MM), k0 = io.ticks();
  const int16_t f0 = us[FRONT];
  float tAll = 0;
  uint8_t close = 0;
  bool near = false;
  for (uint8_t leg = 0; leg < 4 && !near && io.ticks() - k0 < want; ++leg) {
    if (!io.driveStart(100, 0, wcomp[0][1], START_FWD_PWM, FOCUS_FRONT)) return fail(RL_F("stopped"));
    float t = 0;
    for (;;) {
      int16_t z[NUS];
      float dt;
      if (!io.drivePoll(z, &dt)) { io.driveEnd(); return fail(RL_F("stopped")); }
      t += dt * 1000;
      if (z[FRONT] != NOT_PINGED) { close = z[FRONT] > 0 && z[FRONT] < START_FWD_NEAR ? close + 1 : 0; if (close >= 2) { near = true; break; } }
      if (io.ticks() - k0 >= want || t >= START_FWD_LEG_MS) break;
    }
    if (!io.driveEnd()) return fail(RL_F("stopped"));
    tAll += t;
    TRY(measure(us));
    TRY(hold(us));   // back onto the start heading
  }
  TRY(measure(us));
  const uint32_t got = io.ticks() - k0;
  io.event(RL_F("start_fwd"), i16(got), i16(got / ENC_TICKS_PER_MM), i16(tAll / 10), near);
  io.event(RL_F("start_fwd_front"), f0, us[FRONT]);
  return true;
}

bool Runner::aimStart(int16_t us[NUS], bool* done) {
  *done = false;
  float y0, y;
  for (uint8_t t = 0; t < 3 && us[FRONT] <= 0; ++t) TRY(measure(us));   // a lost echo: measure again
  if (!io.yaw(&y0)) return true;
  // Steep start (3 Nov field, IMG_4001: R45 676, front 1726): one 45 deg beam is near square to the main wall
  // and reads its perpendicular distance, while the front, too far off square for an echo straight back,
  // reads the far corner or nothing. Sim: that ratio is >= 0.75 while the front still sees the main wall,
  // 0.35-0.49 once it reads the corner.
  for (uint8_t t = 0; STEEP_ON > 0.5f && t < 2 && (us[L45] <= 0 || us[R45] <= 0); ++t) TRY(measure(us));   // one stray echo voids a 45
  io.event(RL_F("start_read"), us[FRONT], us[L45], us[R45]);
  // checked on the readings now and on those before the start creep (sim: a 45 deg beam can miss its echo
  // on one of them; the creep keeps the line to N, so a steep start seen there is still one)
  for (uint8_t which = 0; which < 2; ++which) {
    const int16_t* q = which ? pre : us;
    const int16_t l = q[L45], r = q[R45];
    const bool useL = l > 0 && (r <= 0 || l < r);
    const int16_t s45 = useL ? l : r;
    if (STEEP_ON > 0.5f && s45 > 0 && s45 < STEEP_NEAR_MAX && (q[FRONT] <= 0 || s45 < STEEP_RATIO * q[FRONT])) {
      io.event(RL_F("steep_start"), q[FRONT], l, r, which);
      return steepStart(y0, useL ? 1 : -1, us, done);
    }
  }
  if (us[FRONT] <= 0) return true;
  const float f0 = us[FRONT];
  int8_t dir = 0;
  float f1 = 0, delta = 0, fp[2] = {0, 0}, dp[2] = {0, 0};   // probe readings / turned angles, + side and - side
  for (int8_t s = 1; s >= -1 && !dir; s -= 2) {
    TRY(rotateTo(y0 + s * START_PROBE, ROT_TOL, false));
    TRY(measure(us));
    if (us[FRONT] <= 0) TRY(measure(us));
    if (!io.yaw(&y)) return true;
    io.event(RL_F("start_probe"), s, i16(f0), us[FRONT], i16((y - y0) * 10));
    uint8_t i = s > 0 ? 0 : 1;
    fp[i] = us[FRONT];
    dp[i] = fabsf(y - y0);
    if (us[FRONT] > 0 && us[FRONT] < f0 - START_DROP * f0 && dp[i] > 6) { dir = s; f1 = fp[i]; delta = dp[i]; }
  }
  // neither fell 2 % (a wide beam flattens the reading near square): the two sides' difference still shows
  // where the normal is (3 Nov sim, ~21 deg starts: -19 mm one way, +70 mm the other)
  if (!dir && fp[0] > 0 && fp[1] > 0 && dp[0] > 6 && dp[1] > 6 && fabsf(fp[0] - fp[1]) > START_ASYM) {
    uint8_t i = fp[0] < fp[1] ? 0 : 1;
    dir = i ? -1 : 1; f1 = fp[i]; delta = dp[i];
  }
  // angle of the start heading off the normal (deg), the centre's distance from the wall
  float bestA = 0, bestE = 1e9f, dc = 0;
  if (dir) for (float a = delta + 1; a <= 80; a += 0.25f) {
    float off = a - START_CONE;
    float d = f0 * cosf((off > 0 ? off : 0) * DEG) + FRONT_FWD * cosf(a * DEG);
    float e = fabsf(startModel(a, d, a - delta) - f1);
    if (e < bestE) { bestE = e; bestA = a; dc = d; }
  }
  setGoal(y0);
  TRY(rotateTo(goal, ROT_TOL, false));
  TRY(measure(us));
  for (uint8_t t = 0; t < 2 && us[FRONT] <= 0; ++t) TRY(measure(us));
  if (us[FRONT] <= 0) us[FRONT] = i16(f0);   // back at the start heading: the start's reading holds
  io.event(RL_F("start_angle"), dir, i16(bestA * 10), i16(dc), i16(bestE));
  // A steep start (3 Nov field, IMG_4001: ~57 deg out on the arc, pointing 65 deg off square): the front gets
  // no echo straight back from the main wall there; it reads the far corner instead (1726 mm, the same 12 deg
  // either way), so the angle solve, which assumes the main wall, cannot fit. Also with no probe side but the
  // two 45 deg beams far apart (one square-ish to the main wall) and a front reading longer than the field.
  if (STEEP_ON > 0.5f && dir && (bestE > START_FIT_MAX || bestA >= 79)) return steepStart(y0, dir, us, done);
  if (!dir || bestA < START_MIN_DEG || bestE > START_FIT_MAX) return true;
  const int16_t* n = cal.ref[N];
  float ca = cosf(bestA * DEG), dsNow = dc - FRONT_FWD * ca, dsN = n[FRONT] + FRONT_FWD - FRONT_FWD * ca;
  if (dsNow < dsN + START_MIN_TRAVEL && STEEP_ON <= 0.5f) return true;
  if (dsNow < dsN + START_MIN_TRAVEL) {   // already near N's line (the walk-first got it there): square up, no walk
    setGoal(y0 + dir * bestA);
    TRY(rotateTo(goal, ROT_TOL, false));
    TRY(measure(us));
    io.event(RL_F("start_square"), us[FRONT], us[LSIDE], us[RSIDE]);
    *done = true;
    return true;
  }
  float target = us[FRONT] * dsN / dsNow;
  io.event(RL_F("start_walk"), us[FRONT], i16(target), i16(dsNow), i16(dsN));
  for (uint8_t leg = 0; leg < 12; ++leg) {
    for (uint8_t t = 0; t < 3 && us[FRONT] <= 0; ++t) TRY(measure(us));   // one stray echo voids a reading
    // the front lost the main wall on the way (sim, ~44 deg off square: after 100 mm; the walk then stopped
    // and the robot went in beside the side wall): finish on the 45 deg beam facing the wall
    if (us[FRONT] <= 0 && STEEP_ON > 0.5f && bestA >= 25) { TRY(walk45(bestA, dir, us)); break; }
    if (us[FRONT] <= 0 || us[FRONT] - target <= 15) break;
    TRY(driveForward(target, START_LEG_MS, START_PWM, us));
    if (cornered) { TRY(recover(SET_ALL, us)); break; }   // a wall close: back off, then square anyway
    TRY(hold(us));
  }
  setGoal(y0 + dir * bestA);   // square to the main wall
  TRY(rotateTo(goal, ROT_TOL, false));
  TRY(measure(us));
  io.event(RL_F("start_square"), us[FRONT], us[LSIDE], us[RSIDE]);
  *done = true;
  return true;
}

// GAME PLAN (2 Nov, user; second field day):
//  1 creep straight at the start heading until the front reads ~50 cm (a 45 deg beam under 10 cm stops it)
//  2 straight on to N's front reading: once both side walls are tracked, their change against the
//    front's gives the heading (skewFrom), turned out; combined omnidirectional moves onto N. The front
//    at N flips between two echoes ~25 mm apart (2 Nov), so forward is done within 15 mm, sideways 8 mm
//    (the short heading-check move was dropped: over 8 cm the side noise gave a 13 deg error)
//  3 read the colour; 4 turn 90 deg in one motion (gyro)
//  5 no settling on the turned capture (it was 11 cm off N's newer capture and cost ~1 min): straight
//    forward to 180 mm short of the box, the front and 45 deg beams guarding the walls, the side on the
//    main wall giving the heading; 6 parkIn: sideways settled there, one straight creep in, small
//    single-axis fixes (front 5 mm, side 8 mm, heading 1.5 deg)
// An imperfect position never ends the run: each stage logs its accuracy and the run goes on.
bool Runner::run(uint8_t first, uint8_t last, int8_t forceTurn) {
  why = 0;
  int16_t us[NUS], a[NUS];
  float y, rg = 0;
  TRY(measure(us));
  if (!io.yaw(&y)) return fail(RL_F("gyro_not_zeroed"));
  setGoal(y);   // placed pointing at the wall: hold this heading while creeping
  const int16_t* n = cal.ref[N];
  cornerMm = 100;
  if (first <= 1 && last >= 1) {
    io.event(RL_F("step"), 1);
    wallPhase = true;
    bool aimed;
    TRY(startForward(us));                                             // 25 cm forward on the encoders first (user, 3 Nov evening)
    TRY(startNudge(us));                                               // (off: START_NUDGE 0)
    TRY(aimStart(us, &aimed));                                         // steep start: walk the line to N
    if (!aimed) TRY(faceWall(us));
    if (!aimed) TRY(walkTo(APPROACH_MM, us));                          // 1
    if (cornered) TRY(recover(SET_ALL, us));                           //   stopped at a wall: back off, re-localize
  }
  if (first <= 2 && last >= 2) {
    io.event(RL_F("step"), 2);
    wallPhase = true;
    if (us[FRONT] > 0 && fabsf(float(us[FRONT] - n[FRONT])) > 15)
      TRY(driveForward(n[FRONT], CREEP_MAX_MS, DRIVE_PWM, us));        // 2: straight on to N's distance
    // Heading at N (3 Nov sim): the side walls end ~48 cm from the main wall, so on the drive in they are
    // seen only over its last few cm and that fit is noise (it turned the robot 6 deg the wrong way); the
    // old check backed up 12 cm, out of their span. Instead a short straight move toward the main wall and
    // back, both inside the side walls, each leg's side-wall fit turned out (skewFrom / driveSkew).
    bool known = false;
    for (uint8_t tryN = 0; tryN < 2 && !known && us[FRONT] > 0; ++tryN) {
      io.event(RL_F("heading_move"), tryN);
      bool k1 = false, k2 = false;
      skewMode = 1 + SET_ALL;
      memcpy(a, us, sizeof(a));
      TRY(driveForward(n[FRONT] - HEADING_MOVE, DRIVE_MAX_MS, DRIVE_PWM, us));
      if (!cornered) TRY(straighten(SET_ALL, a, us, &k1, N_SKEW_MAX));
      memcpy(a, us, sizeof(a));
      if (us[FRONT] > 0) TRY(driveForward(n[FRONT], DRIVE_MAX_MS, DRIVE_PWM, us));
      TRY(straighten(SET_ALL, a, us, &k2, N_SKEW_MAX));
      skewMode = 0;
      cornered = false;
      known = k1 || k2;
    }
    TRY(align(N, SET_ALL, 95, 15, 8, 6, us));                          //    combined moves onto N
  }
  turnDir = forceTurn;
  if (first <= 3 && last >= 3) {
    io.event(RL_F("step"), 3);
    wallPhase = false;
    TRY(measure(us, &rg));                                             // 3
    float g = cal.greenMilli / 1000.0f, r = cal.redMilli / 1000.0f;
    if (rg <= 0 || g <= 0 || r <= g) return fail(RL_F("no_colour_reading"));
    float split = sqrtf(g * r);
    float conf = clampf(fabsf(logf(rg / split)) / (logf(r / g) / 2), 0, 1);
    turnDir = rg > split ? -1 : 1;   // red turns right, green turns left
    io.event(RL_F("colour"), i16(rg * 1000), i16(conf * 100), turnDir);
    TRY(turnOnce(turnDir > 0));                                        // 4: gyro 90 deg
    Pose turned = turnDir > 0 ? N_LEFT : N_RIGHT;                      // 5: fit the turned capture on its 3 sensors
    uint8_t set = turnDir > 0 ? SET_LEFT_TURN : SET_RIGHT_TURN;        //    (front, side and 45 deg on the main wall)
    TRY(align(turned, set, 95, 10, 10, 4, us));                        //    position
    TRY(fitHeading(turned, set, us));                                  //    heading
    TRY(align(turned, set, 95, 10, 10, 4, us));                        //    position again
  }
  if (first <= 4 && last >= 4) {
    io.event(RL_F("step"), 4);
    inPark = true;
    if (turnDir == 0) return fail(RL_F("no_turn_direction"));
    Pose box = turnDir > 0 ? BOX_LEFT : BOX_RIGHT;
    uint8_t set = turnDir > 0 ? SET_LEFT_TURN : SET_RIGHT_TURN;
    beamSlope[set] = 0;   // learned afresh from the box approach's own turn
    TRY(measure(us));
    const int16_t* tr = cal.ref[turnDir > 0 ? N_LEFT : N_RIGHT];
    if (tr[FRONT] != NONE) TRY(faceExpected(tr[FRONT], us));           //    the front must see the side wall
    memcpy(a, us, sizeof(a));
    skewMode = 1 + set;
    TRY(walkTo(int16_t(cal.ref[box][FRONT] + PARK_APPROACH), us));                       // 5: straight, slowly, to 180 mm short
    skewMode = 0;
    if (cornered) TRY(recover(set, us));                               //    stopped at a wall: back off, re-localize
    else TRY(straighten(set, a, us, 0, PARK_SKEW_MAX));                            //    heading from the main wall (<= 8 deg: 2 Nov a
                                                                       //    28 deg misfit drove the robot off the field)
    TRY(parkIn(box, set, us));                                         // 6: sideways in the open, straight in, small fixes
    if (!PARK_ORIG) TRY(finalAdjust(box, set, us));                   // 7: last adjust, time-boxed
    TRY(measure(us));
    float yy;
    io.event(RL_F("parked"), us[FRONT], cal.ref[box][FRONT], io.yaw(&yy) && goalSet ? i16((yy - goal) * 10) : NONE);
  }
  io.event(RL_F("done"));
  return true;
}

#ifndef __AVR__
// Host only: set a tunable by name (the simulator's PARAMS="NAME=value,..."); false = unknown name.
bool setTunable(const char* name, float v) {
  struct { const char* n; float* f; } fl[] = {
    {"CORNER_MARGIN", &CORNER_MARGIN}, {"CORNER_ZONE", &CORNER_ZONE}, {"PARK_DEAD_Y", &PARK_DEAD_Y},
    {"HEADING_MOVE", &HEADING_MOVE}, {"N_SKEW_MAX", &N_SKEW_MAX}, {"PARK_APPROACH", &PARK_APPROACH},
    {"PARK_SKEW_MAX", &PARK_SKEW_MAX}, {"PARK_FINAL_SKEW_MAX", &PARK_FINAL_SKEW_MAX}, {"PARK_RECOVER_MAX", &PARK_RECOVER_MAX}, {"FINAL_MS", &FINAL_MS}, {"FINAL_CYCLE_MS", &FINAL_CYCLE_MS},
    {"FINAL_BACK", &FINAL_BACK}, {"FINAL_SKEW_MIN", &FINAL_SKEW_MIN}, {"FINAL_MAX_TURN", &FINAL_MAX_TURN}, {"FINAL_TOL_X", &FINAL_TOL_X},
    {"FINAL_TOL_Y", &FINAL_TOL_Y}, {"FINAL_HEAD", &FINAL_HEAD}, {"FINAL_RAMP", &FINAL_RAMP},
    {"START_PROBE", &START_PROBE}, {"START_CONE", &START_CONE}, {"START_MIN_DEG", &START_MIN_DEG},
    {"START_LEG_MS", &START_LEG_MS}, {"START_DROP", &START_DROP}, {"START_ASYM", &START_ASYM},
    {"START_FIT_MAX", &START_FIT_MAX}, {"START_MIN_TRAVEL", &START_MIN_TRAVEL},
    {"START_STEP", &START_STEP}, {"START_NUDGE", &START_NUDGE}, {"START_NUDGE_LEG_MS", &START_NUDGE_LEG_MS}, {"START_NUDGE_LEGS", &START_NUDGE_LEGS}, {"START_NUDGE_STEEP", &START_NUDGE_STEEP}, {"START_FWD_MM", &START_FWD_MM}, {"ENC_TICKS_PER_MM", &ENC_TICKS_PER_MM}, {"START_NUDGE_RAMP", &START_NUDGE_RAMP}, {"START_NUDGE_NEAR", &START_NUDGE_NEAR}, {"START_SEARCH_MAX", &START_SEARCH_MAX}, {"STEEP_ON", &STEEP_ON}, {"STEEP_RATIO", &STEEP_RATIO}, {"STEEP_NEAR_MAX", &STEEP_NEAR_MAX}};
  for (auto& e : fl) if (!strcmp(e.n, name)) { *e.f = v; return true; }
  if (!strcmp(name, "START_PWM")) { START_PWM = uint8_t(v + 0.5f); return true; }
  struct { const char* n; uint8_t* p; } u8[] = {{"PWM_X", &PWM_X}, {"PWM_Y", &PWM_Y}, {"CREEP_FLOOR", &CREEP_FLOOR}, {"DRIVE_PWM", &DRIVE_PWM},
    {"SIDE_PWM", &SIDE_PWM}, {"ROT_PWM_C", &ROT_PWM_C}, {"ROT_PWM_F", &ROT_PWM_F}, {"FIX_PWM", &FIX_PWM}};
  for (auto& e : u8) if (!strcmp(e.n, name)) { *e.p = uint8_t(v + 0.5f); return true; }
  if (!strcmp(name, "CREEP_SLOW")) { CREEP_SLOW = v; return true; }
  struct { const char* n; uint8_t* p; } u8b[] = {{"WALL_PWM", &WALL_PWM}, {"WALL_WALK_PWM", &WALL_WALK_PWM}, {"WALL_FLOOR", &WALL_FLOOR},
    {"PARK_PWM", &PARK_PWM}, {"PARK_FLOOR", &PARK_FLOOR}, {"PARK_SIDE_PWM", &PARK_SIDE_PWM}};
  for (auto& e : u8b) if (!strcmp(e.n, name)) { *e.p = uint8_t(v + 0.5f); return true; }
  if (!strcmp(name, "WALL_SLOW")) { WALL_SLOW = v; return true; }
  if (!strcmp(name, "START_FWD_PWM")) { START_FWD_PWM = uint8_t(v + 0.5f); return true; }
  if (!strcmp(name, "PARK_SLOW")) { PARK_SLOW = v; return true; }
  if (!strcmp(name, "APPROACH_MM")) { APPROACH_MM = int16_t(v + 0.5f); return true; }
  return false;
}
#endif
}  // namespace rl

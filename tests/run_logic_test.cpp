// Host test of the on-Mega tournament controller (firmware/hardware_probe_mega/src/run_logic.*)
// in a simulated arena. Geometry approximates the 30 Oct practice field. The robot is simulated
// in 5 ms steps with the behaviour of the real firmware and chassis, every constant randomized
// per run (battery, floor, robot) so the controller has to learn it rather than rely on it:
//   - firmware soft start: outputs ramp 8 PWM per 10 ms (rampMotors); a pulse ramps down the same
//     way, then short-brakes; a drive short-brakes 120 ms (MegaIo::pulse / driveEnd)
//   - dead band (no motion below a breakaway PWM per axis), first-order inertia, coasting after
//     release (forward rolls far: 2 Nov ~20 cm from 380 mm/s), braking stops in a few cm
//   - strafes turn the robot CCW, forward motion turns it CW, in proportion to distance
//   - gyro read only while stopped; its FIFO (yaw only) holds 2.5 s, a longer motor-on stretch loses
//     frames (lost() changes, the angle jumps); scale error up to +/-2 %, arbitrary zero
//   - ultrasonics: +/-12 deg beam cone, 4 mm noise, missing and spurious echoes; a ping takes
//     its echo time (18 ms timeout when nothing returns)
//   - calibration captures carry their own few-mm noise
// It checks the logic, not the field: real accuracy needs field runs.
// Build/run: tools/verify_run_logic.sh [v|q] [arc_deg] [aim_deg] [verbose_run]
//   env RUNS (default 24), SEED (default 3848); RUNS=480 for an accuracy figure.
#include "../firmware/hardware_probe_mega/src/run_logic.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double WALL_Y = 1154;               // main wall, P at (0, 0) facing +y
static const double N_Y = WALL_Y - 418;          // N: the front reads 268 there
static const double PI = 3.14159265358979;
static const double HALF_LEN = 160, HALF_W = 120;   // robot body from its centre (front sensors at 150)
struct Mount { double fwd, left, deg; };
static const Mount MOUNT0[5] = {{150, 95, 45}, {150, -95, -45}, {150, 0, 0}, {0, 115, 90}, {0, -115, -90}};
// Arena: the fixed practice field. Side walls at +/-SIDE_X from SIDE_FROM to the main wall.
static const double SIDE_X = 770, SIDE_FROM = N_Y - 62, CONE = 12;   // CONE: beam half-width, deg
static const Mount* const MOUNT = MOUNT0;
static bool pulseCoast = false;   // env PULSE=coast: pulses end by release (firmware before 2 Nov)

static double urand() { return rand() / (double)RAND_MAX; }
static double uni(double lo, double hi) { return lo + (hi - lo) * urand(); }
static double gauss() { return sqrt(-2 * log(urand() + 1e-12)) * cos(2 * PI * urand()); }

struct Seg { double x1, y1, x2, y2; };
static const Seg WALLS[3] = {{-SIDE_X, WALL_Y, SIDE_X, WALL_Y},
                             {-SIDE_X, SIDE_FROM, -SIDE_X, WALL_Y},
                             {SIDE_X, SIDE_FROM, SIDE_X, WALL_Y}};

// Distance along a ray to the nearest wall, or -1. x grows to the right; heading CCW from +y.
static double ray(double px, double py, double ang) {
  double dx = -sin(ang), dy = cos(ang), best = -1;
  for (const Seg& s : WALLS) {
    double ex = s.x2 - s.x1, ey = s.y2 - s.y1, den = dx * ey - dy * ex;
    if (fabs(den) < 1e-9) continue;
    double t = ((s.x1 - px) * ey - (s.y1 - py) * ex) / den;
    double u = ((s.x1 - px) * dy - (s.y1 - py) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && (best < 0 || t < best)) best = t;
  }
  return best;
}

struct Pose { double x, y, psi; };   // psi radians, CCW

static double senseOne(const Pose& p, int k) {
  const Mount& m = MOUNT[k];
  double sx = p.x - m.fwd * sin(p.psi) - m.left * cos(p.psi);
  double sy = p.y + m.fwd * cos(p.psi) - m.left * sin(p.psi);
  double best = -1;
  for (double c = -CONE; c <= CONE + 0.01; c += CONE / 4) {   // beam cone
    double d = ray(sx, sy, p.psi + (m.deg + c) * PI / 180);
    if (d > 0 && (best < 0 || d < best)) best = d;
  }
  return best > 2500 || best < 20 ? -1 : best;
}

// One noisy ping: 4 mm noise, 5 % no echo, `spurious` chance of a stray echo.
static double ping(const Pose& p, int k, double spurious) {
  double d = senseOne(p, k);
  if (d > 0 && urand() < 0.05) return -1;
  if (urand() < spurious) return uni(100, 1500);
  return d > 0 ? d + 4 * gauss() : -1;
}

// Per-run chassis constants (battery, floor, this robot), drawn once per run.
struct Chassis {
  double g[3];      // steady speed per PWM above breakaway: mm/ms (x, y), deg/ms (w)
  double stall[3];  // breakaway PWM per axis
  double coast[3];  // s: speed decay after release
  double tauAcc, tauBrake;
  double cplY, cplX;   // yaw per mm: strafe turns CCW, forward turns CW (deg/mm)
  double gyroScale, gyroZero;
  void draw() {
    // Field-matched centres: forward 75 PWM ~ 380 mm/s (2 Nov); fine rotation 80 PWM ~ 7 deg per
    // 120 ms pulse; strafe 100 PWM ~ 250 mm/s.
    g[0] = 0.0127 * uni(0.8, 1.25); g[1] = 0.0060 * uni(0.75, 1.3); g[2] = 0.0033 * uni(0.8, 1.25);
    stall[0] = uni(40, 50); stall[1] = uni(52, 66); stall[2] = uni(42, 54);
    coast[0] = uni(0.3, 0.55); coast[1] = uni(0.05, 0.12); coast[2] = uni(0.04, 0.09);
    tauAcc = uni(0.04, 0.08); tauBrake = uni(0.02, 0.04);
    cplY = 0.07 * uni(0.5, 1.5); cplX = 0.015 * uni(0.5, 1.5);
    gyroScale = 1 + uni(-0.02, 0.02); gyroZero = 30 * gauss();
  }
};

class Sim : public rl::Io {
 public:
  Pose p;
  Chassis c;
  double padRg = 1.66;   // colour under the sensor at N
  bool verbose = false, collided = false;
  int moves = 0;
  long simMs = 0;
  uint16_t lostCount = 0;
  double gyroJump = 0;   // accumulated angle error from lost FIFO frames

  // Motion state: command mix (normalized so the largest wheel is 1), ramped level u, velocities.
  double ax = 0, ay = 0, aw = 0, u = 0, target = 0, v[3] = {0, 0, 0}, moveGain = 1;
  enum Mode { OFF, RUN, BRAKE } mode = OFF;
  double motorOnMs = 0;   // this motor-on stretch, for the gyro FIFO

  void setMix(int8_t x, int8_t y, int8_t w) {
    double fl = x - y - w, fr = x + y + w, rl = x + y - w, rr = x - y + w, peak = 1;
    const double wheel[4] = {fl, fr, rl, rr};
    for (double q : wheel) peak = fabs(q) > peak ? fabs(q) : peak;
    ax = x / peak; ay = y / peak; aw = w / peak;
  }
  void step(double dt) {   // dt ms
    simMs += long(dt);
    if (mode == RUN) {
      u += u < target ? (u + 0.8 * dt > target ? target - u : 0.8 * dt) : (u - 0.8 * dt < target ? target - u : -0.8 * dt);
      const double a[3] = {ax, ay, aw};
      for (int k = 0; k < 3; ++k) {
        double drive = u * fabs(a[k]) - c.stall[k];
        double vss = drive > 0 ? c.g[k] * moveGain * drive * (a[k] > 0 ? 1 : -1) : 0;
        if (drive <= 0 && fabs(v[k]) < 1e-3) v[k] = 0;   // static friction holds a still robot
        else v[k] += (vss - v[k]) * (1 - exp(-dt / (c.tauAcc * 1000)));
      }
      motorOnMs += dt;
    } else {
      for (int k = 0; k < 3; ++k) v[k] *= exp(-dt / ((mode == BRAKE ? c.tauBrake : c.coast[k]) * 1000));
      if (mode == BRAKE) motorOnMs += dt;
    }
    double fwd = v[0] * dt, left = v[1] * dt;
    p.x += -fwd * sin(p.psi) - left * cos(p.psi);
    p.y += fwd * cos(p.psi) - left * sin(p.psi);
    p.psi += (v[2] * dt + c.cplY * left - c.cplX * fwd) * PI / 180;
    checkCollision();
  }
  void motorsOff() {   // release: the FIFO is drained; a stretch over 0.85 s lost frames
    mode = OFF;
    if (motorOnMs > 2500) { ++lostCount; gyroJump += 1.5 * gauss(); }   // yaw-only FIFO: 1024 B / 400 B/s
    motorOnMs = 0;
  }
  void run(double ms) { for (double t = 0; t < ms; t += 5) step(5); }
  void checkCollision() {
    for (int i = 0; i < 4; ++i) {
      double f = i < 2 ? HALF_LEN : -HALF_LEN, l = i % 2 ? HALF_W : -HALF_W;
      double cx = p.x - f * sin(p.psi) - l * cos(p.psi), cy = p.y + f * cos(p.psi) - l * sin(p.psi);
      if (cy >= WALL_Y || (cy >= SIDE_FROM && fabs(cx) >= SIDE_X)) collided = true;
    }
  }

  bool measure(int16_t us[rl::NUS], float* rg) override {
    // The scan in progress is skipped, then three 5 Hz scans (the robot may still be coasting).
    run(100);
    double scans[3][5];
    for (int s = 0; s < 3; ++s) {
      run(200);
      for (int k = 0; k < 5; ++k) scans[s][k] = ping(p, k, 0.02);
    }
    for (int k = 0; k < 5; ++k) {
      double q[3];
      int n = 0;
      for (int s = 0; s < 3; ++s) if (scans[s][k] > 0) q[n++] = scans[s][k];
      double lo = 1e9, hi = -1e9;
      for (int i = 0; i < n; ++i) { lo = q[i] < lo ? q[i] : lo; hi = q[i] > hi ? q[i] : hi; }
      if (n < 2 || hi - lo > 40) { us[k] = -1; continue; }
      double med = n == 3 ? q[0] + q[1] + q[2] - lo - hi : (q[0] + q[1]) / 2;
      us[k] = int16_t(med + 0.5);
    }
    double cx = p.x - 150 * sin(p.psi), cy = p.y + 150 * cos(p.psi);   // colour sensor, front
    *rg = float(hypot(cx - 0, cy - (N_Y + 150)) < 60 ? padRg : 1.05);
    return moves < 900;
  }
  // Firmware pulse: soft start to pwm, hold until ms, ramp down at the same rate, brake 60 ms,
  // release, settle 250 ms.
  bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint16_t ms) override {
    ++moves;
    setMix(x, y, w);
    moveGain = 1 + 0.08 * gauss();
    mode = RUN; u = 0; target = pwm;
    run(ms);
    if (pulseCoast) { motorsOff(); run(350); return moves < 900; }
    target = 0;
    while (u > 0) step(5);
    mode = BRAKE; run(60);
    motorsOff();
    run(250);
    return moves < 900;
  }
  // Continuous sweep: like a pulse (soft start, hold, ramp down, brake, settle), the front pinged every
  // 25 ms + echo time; each ping's heading comes from the gyro's buffered samples (exact to the
  // ~5 ms sample time: modelled as 0.1 deg noise).
  bool sweep(int8_t dir, uint8_t pwm, uint16_t ms, float* yaws, int16_t* fronts, uint8_t maxN, uint8_t* n) override {
    ++moves;
    *n = 0;
    setMix(0, 0, dir * 100);
    moveGain = 1 + 0.08 * gauss();
    mode = RUN; u = 0; target = pwm;
    double t = 0, nextPing = 0;
    auto pingNow = [&]() {
      if (*n >= maxN) return;
      double d = ping(p, rl::FRONT, 0.03);
      float y;
      yaw(&y);
      yaws[*n] = y + float(0.1 * gauss());
      fronts[*n] = d > 0 ? int16_t(d + 0.5) : -1;
      ++*n;
      nextPing = t + 25 + (d > 0 ? d / 171.5 : 18);
    };
    for (; t < ms; t += 5) { if (t >= nextPing) pingNow(); step(5); }
    target = 0;
    while (u > 0) { if (t >= nextPing) pingNow(); step(5); t += 5; }
    mode = BRAKE;
    for (double b = 0; b < 60; b += 5) { if (t >= nextPing) pingNow(); step(5); t += 5; }
    motorsOff();
    run(250);
    return moves < 900;
  }
  // Continuous drive: one ping per poll >= 25 ms apart in the firmware's order (front every other
  // poll, then LSIDE, R45, RSIDE, L45), a ping lasting its echo time; driveEnd brakes 120 ms.
  int drvPolls = 0;
  int drvFocus = 0;
  bool driveStart(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint8_t focus) override {
    ++moves;
    drvPolls = 0;
    drvFocus = focus;
    moveGain = 1 + 0.08 * gauss();
    setMix(x, y, w);
    mode = RUN; u = 0; target = pwm;
    return moves < 900;
  }
  bool driveSet(int8_t x, int8_t y, int8_t w, uint8_t pwm) override { setMix(x, y, w); target = pwm; return true; }
  bool drivePoll(int16_t us[rl::NUS], float* dt) override {
    static const int OTHERS[4] = {rl::LSIDE, rl::R45, rl::RSIDE, rl::L45};
    static const int SIDES[5] = {rl::LSIDE, rl::RSIDE, rl::LSIDE, rl::RSIDE, rl::FRONT};
    int k = drvFocus == rl::FOCUS_SIDES ? SIDES[drvPolls % 5] : drvPolls % 2 ? OTHERS[(drvPolls / 2) % 4] : rl::FRONT;
    ++drvPolls;
    run(25);
    double d = ping(p, k, 0.03);
    double pingMs = d > 0 ? d / 171.5 : 18;
    run(pingMs);
    for (int i = 0; i < rl::NUS; ++i) us[i] = rl::NOT_PINGED;
    us[k] = d > 0 ? int16_t(d + 0.5) : -1;
    *dt = float((25 + 5 * ceil(pingMs / 5)) / 1000);
    return moves < 900;
  }
  bool driveEnd() override {
    mode = BRAKE; run(120);
    motorsOff();
    run(350);
    return true;
  }
  bool yaw(float* deg) override {
    *deg = float((p.psi * 180 / PI) * c.gyroScale + c.gyroZero + gyroJump + 0.05 * gauss());
    return true;
  }
  uint16_t lost() override { return lostCount; }
  Pose atStep[5];
  double sqY0True = 0, sqLast = 0, firstSq = 0; int sqCount = 0; bool sqStarted = false;
  // The square event reports psi - y0 (gyro, 0.1 deg); y0 = gyro at the start of squareHere, taken
  // from the first sweep event of that call (sweep a = (y - y0)*10).
  double squareEst(int16_t a) {
    // true heading the controller now believes is the wall normal = trueAtY0 + a/10 / scale
    return sqY0True + a / 10.0 / c.gyroScale;
  }
  bool reachedStep[5] = {false, false, false, false, false};
  void event(const char* name, int16_t a, int16_t b, int16_t cc, int16_t d) override {
    if (!strcmp(name, "step") && a >= 1 && a <= 4) { atStep[a] = p; reachedStep[a] = true; }
    if (!strcmp(name, "sweep") && !sqStarted) {
      sqStarted = true;
      sqY0True = p.psi * 180 / PI - a / 10.0 / c.gyroScale;   // true heading when y0 was read
    }
    if (!strcmp(name, "square")) {
      sqStarted = false;   // estimate error: the fitted normal vs the true wall normal (heading 0)
      sqLast = squareEst(a);
      if (!sqCount) firstSq = sqLast;
      ++sqCount;
    }
    if (verbose) printf("    %-28s %6d %6d %6d %6d   pose x %5.0f y %5.0f psi %6.1f  t %5.1fs\n", name, a, b, cc, d, p.x, p.y, p.psi * 180 / PI, simMs / 1000.0);
  }
};

static void capture(const Pose& p, int16_t ref[rl::NUS], int dropSide) {
  for (int k = 0; k < 5; ++k) {
    double d = senseOne(p, k);
    ref[k] = d > 0 ? int16_t(d + 3 * gauss() + 0.5) : rl::NONE;   // a capture is an average of noisy readings
  }
  if (dropSide >= 0) ref[dropSide] = rl::NONE;   // start-facing side: no field wall (upload rule)
}

int main(int argc, char** argv) {
  bool verbose = argc > 1 && argv[1][0] == 'v';
  srand(getenv("SEED") ? atoi(getenv("SEED")) : 3848);
  int nRuns = getenv("RUNS") ? atoi(getenv("RUNS")) : 24;
  pulseCoast = getenv("PULSE") && !strcmp(getenv("PULSE"), "coast");
  int passed = 0, runs = 0;
  double arc = argc > 2 ? atof(argv[2]) : 20, aim = argc > 3 ? atof(argv[3]) : 15;
  int verboseRun = argc > 4 ? atoi(argv[4]) : 0;
  const int MAXR = 64;
  const char* reasons[MAXR];
  int reasonCount[MAXR], nReasons = 0;
  double sumT = 0, stPsi[5] = {}, stX[5] = {}, stY[5] = {};
  int stN[5] = {}, stPsiBad[5] = {}, stPosBad[5] = {};
  for (int i = 0; i < nRuns; ++i) {
    // The robot that captured the calibration is the robot that runs: same field, fresh noise.
    const double BOX_X = SIDE_X - MOUNT[rl::FRONT].fwd - 47;   // parked facing the side wall, front reading 47
    rl::Calib cal;
    capture({0, 0, 0}, cal.ref[rl::P], -1);
    capture({0, N_Y, 0}, cal.ref[rl::N], -1);
    capture({0, N_Y, PI / 2}, cal.ref[rl::N_LEFT], rl::LSIDE);
    capture({-BOX_X, N_Y, PI / 2}, cal.ref[rl::BOX_LEFT], rl::LSIDE);
    capture({0, N_Y, -PI / 2}, cal.ref[rl::N_RIGHT], rl::RSIDE);
    capture({BOX_X, N_Y, -PI / 2}, cal.ref[rl::BOX_RIGHT], rl::RSIDE);
    cal.greenMilli = 830;
    cal.redMilli = 1660;
    // Start on the arc ~1.2 m from the light within +/-arc deg (field-tested starts were ~30-40 cm
    // either side, about +/-20 deg), pointing at the wall within +/-aim deg.
    double a = (-arc + 2 * arc * urand()) * PI / 180, r = WALL_Y + 100 * urand();
    Sim sim;
    sim.c.draw();
    sim.verbose = verbose && i == verboseRun;
    sim.p = {-r * sin(a), WALL_Y - r * cos(a), a + (-aim + 2 * aim * urand()) * PI / 180};
    sim.padRg = i % 2 ? 0.83 : 1.66;
    Pose start = sim.p;
    rl::Runner runner(sim, cal);
    bool ok = runner.run();
    bool left = runner.turn() > 0;
    double tx = left ? -BOX_X : BOX_X, tpsi = left ? 90 : -90;
    double ex = sim.p.x - tx, ey = sim.p.y - N_Y, epsi = sim.p.psi * 180 / PI - tpsi;
    bool wantLeft = sim.padRg < 1.2;
    bool good = ok && left == wantLeft && fabs(ex) < 30 && fabs(ey) < 30 && fabs(epsi) < 4 && !sim.collided;
    ++runs;
    passed += good;
    for (int st = 2; st <= 4; ++st) if (sim.reachedStep[st]) {
      const Pose& q = sim.atStep[st];
      double want = st == 4 ? (left ? 90 : -90) : 0;
      stN[st]++;
      stPsi[st] += fabs(q.psi * 180 / PI - want);
      if (fabs(q.psi * 180 / PI - want) > 3) stPsiBad[st]++;
      if (st >= 3) { stX[st] += fabs(q.x); stY[st] += fabs(q.y - N_Y); if (fabs(q.x) > 30 || fabs(q.y - N_Y) > 30) stPosBad[st]++; }
    }
    sumT += sim.simMs / 1000.0;
    const char* why = !ok ? runner.reason() : sim.collided ? "collided" : left != wantLeft ? "wrong_box"
                      : good ? "OK" : fabs(epsi) >= 4 ? "parked_heading_off" : "parked_position_off";
    int j = 0;
    while (j < nReasons && strcmp(reasons[j], why)) ++j;
    if (j == nReasons && nReasons < MAXR) { reasons[nReasons] = why; reasonCount[nReasons++] = 0; }
    if (j < MAXR) ++reasonCount[j];
    if (nRuns <= 48 || verbose)
      printf("%2d sq %+5.1f start x %5.0f y %5.0f psi %+5.1f | %s %s | box err x %+5.0f y %+5.0f psi %+5.1f | %3d moves %5.1fs %s%s\n",
             i, sim.firstSq, start.x, start.y, start.psi * 180 / PI, wantLeft ? "green" : "red  ", ok ? "done   " : runner.reason(),
             ex, ey, epsi, sim.moves, sim.simMs / 1000.0, good ? "OK" : "FAIL", sim.collided ? " (collided)" : "");
  }
  printf("outcomes:");
  for (int j = 0; j < nReasons; ++j) printf("  %s %d", reasons[j], reasonCount[j]);
  printf("\nmean run time %.1f s\n", sumT / runs);
  const char* stName[5] = {"", "", "after step 1 (at N, not squared)", "at N (before the colour turn)", "after the turn (before parking)"};
  for (int st = 2; st <= 4; ++st) if (stN[st])
    printf("  %-34s n %4d  |heading| mean %4.1f deg, >3 deg %3d%s", stName[st], stN[st], stPsi[st] / stN[st], stPsiBad[st],
           st >= 3 ? "" : "\n"), st >= 3 ? printf("  |x| %4.0f |y| %4.0f mm, >30 mm %3d\n", stX[st] / stN[st], stY[st] / stN[st], stPosBad[st]) : 0;
  printf("%d/%d simulated runs parked within 30 mm and 4 deg (%.1f %%)\n", passed, runs, 100.0 * passed / runs);
  return passed == runs ? 0 : 1;
}

// Host test of the on-Mega tournament controller (firmware/hardware_probe_mega/src/run_logic.*) in a
// simulated copy of the 3 Nov field. The arena and sensor model are fitted to the 3 Nov calibration
// (calibration/field-2026-11-03a.json) and to the field videos IMG_3994 / IMG_3995:
//   - main wall 1.54 m wide with the light at x = 0; side walls 485 mm deep (video IMG_3995, floor rectified) at x = -745 / +796 (N's side
//     readings 625 / 676 + 120 mm sensor offset)
//   - N 414 mm from the main wall (front reads 264); the turned captures and the boxes 335 mm from it
//     (the side sensor on the main wall reads 215 / 214 / 206 / 213 there); boxes: front reads 52 / 55
//   - smooth panels echo mirror-like: a beam returns from a wall only near square to it; the two inner
//     corners return from any direction inside the beam (a corner reflector: the "45 deg" readings 651 /
//     626 after the turns and 277 / 234 in the boxes are corners); the front sees the main wall up to 75 deg
//   - "45 deg" beams: +-33 deg cone, walls echo only within 10 deg of square, so at N, after the turns and
//     in the boxes they read the inner corners (field: 641 / 716 at N, 651 / 626 turned, 277 / 234 boxed)
//   - per-wheel motor model with the firmware's applyMix deadzone, ramps, braking, coasting; forward
//     drives turn the robot CW, strafes CCW; wheel encoders (channel A, ticks of all four wheels)
//   - gyro read only while stopped; its FIFO holds 2.5 s of motor-on time, rotation beyond that is lost
//   - starts 1.25-1.35 m from the outer semicircle's centre, ~210 mm out from the main wall (video: r ~1 m)
// Every chassis constant is drawn per run (battery, floor). It checks the logic, not the field.
// Build/run: tools/verify_run_logic.sh   env RUNS (default 26 per seed), SEED, TRACE_OUT=file.json
#include "../firmware/hardware_probe_mega/src/run_logic.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

static const double PI = 3.14159265358979;
static const double WALL_Y = 1300;                  // main wall; the light at (0, WALL_Y)
static const double N_Y = WALL_Y - 414;             // N: front 264 + 150 sensor offset
static const double TURN_Y = WALL_Y - 335;          // turned captures and boxes: side 215 + 120
static const double XL = -745, XR = 796;            // side walls
static const double SIDE_DEPTH = getenv("SIDE_DEPTH") ? atof(getenv("SIDE_DEPTH")) : 485, SIDE_FROM = WALL_Y - SIDE_DEPTH;
static const double FRONT_LEN = 150, REAR_LEN = 130, HALF_W = 120;   // body from the turning centre
static const double BOX_LX = XL + FRONT_LEN + 52, BOX_RX = XR - FRONT_LEN - 55;
struct Mount { double fwd, left, deg, cone, inc; };
// front: narrow cone, sees the main wall up to 75 deg; sides: +-12 deg cone, echo within 30 deg of
// square; "45 deg": +-33 deg cone, echo within 10 deg of square (and corners)
static Mount MOUNT[5] = {{150, 120, 45, 33, 10}, {150, -120, -45, 33, 10}, {150, 0, 0, 10, 75},
                               {0, 120, 90, 12, 30}, {0, -120, -90, 12, 30}};
static const double CORNERS[2][2] = {{XL, WALL_Y}, {XR, WALL_Y}};

static double urand() { return rand() / (double)RAND_MAX; }
static double uni(double lo, double hi) { return lo + (hi - lo) * urand(); }
static double gauss() { return sqrt(-2 * log(urand() + 1e-12)) * cos(2 * PI * urand()); }

struct Seg { double x1, y1, x2, y2; };
static const Seg WALLS[3] = {{XL, WALL_Y, XR, WALL_Y}, {XL, SIDE_FROM, XL, WALL_Y}, {XR, SIDE_FROM, XR, WALL_Y}};

// Distance along a ray to the first wall, -1 if none; *inc = incidence (deg from square) at that wall.
static double ray(double px, double py, double ang, double* inc) {
  double dx = -sin(ang), dy = cos(ang), best = -1;
  for (const Seg& s : WALLS) {
    double ex = s.x2 - s.x1, ey = s.y2 - s.y1, den = dx * ey - dy * ex;
    if (fabs(den) < 1e-9) continue;
    double t = ((s.x1 - px) * ey - (s.y1 - py) * ex) / den;
    double u = ((s.x1 - px) * dy - (s.y1 - py) * dx) / den;
    if (!(t > 0 && u >= 0 && u <= 1)) continue;
    if (best < 0 || t < best) {
      double len = sqrt(ex * ex + ey * ey), nx = -ey / len, ny = ex / len;
      best = t;
      *inc = acos(fabs(dx * nx + dy * ny)) * 180 / PI;
    }
  }
  return best;
}

struct Pose { double x, y, psi; };   // psi radians, CCW, 0 = facing the main wall (+y)

static double senseOne(const Pose& p, int k) {
  const Mount& m = MOUNT[k];
  double sx = p.x - m.fwd * sin(p.psi) - m.left * cos(p.psi);
  double sy = p.y + m.fwd * cos(p.psi) - m.left * sin(p.psi);
  double axis = p.psi + m.deg * PI / 180, best = -1;
  for (double c = -m.cone; c <= m.cone + 0.01; c += 2) {   // walls: the part of the cone near square echoes
    double inc, d = ray(sx, sy, axis + c * PI / 180, &inc);
    if (d > 0 && inc <= m.inc && (best < 0 || d < best)) best = d;
  }
  for (const auto& cn : CORNERS) {   // inner corners: a corner reflector inside the cone, if in sight
    double dx = cn[0] - sx, dy = cn[1] - sy, d = hypot(dx, dy);
    double a = atan2(-dx, dy) - axis;
    while (a > PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    double inc, w = ray(sx, sy, atan2(-dx, dy), &inc);
    if (fabs(a) * 180 / PI <= m.cone && w > d - 30 && (best < 0 || d < best)) best = d;
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

// Per-run chassis constants (battery, floor, this robot).
struct Chassis {
  double coast[3], tauAcc, tauBrake, cplY, cplX, gyroScale, gyroZero;
  double wheelStall, wheelGain, strafeEff, mmPerDeg, ticksPerMm;
  void draw() {
    // per wheel (2 Nov field): 75 PWM forward ~380 mm/s, creeps at 41 PWM, in-place turn at 62 PWM ~0.096 deg/ms
    wheelStall = uni(34, 39); wheelGain = 0.38 / (75 - 36.5) * uni(0.85, 1.15);
    strafeEff = uni(0.6, 0.8); mmPerDeg = (wheelGain * (62 - 36.5)) / 0.096 * uni(0.9, 1.1);
    coast[0] = uni(0.3, 0.55); coast[1] = uni(0.05, 0.12); coast[2] = uni(0.04, 0.09);
    tauAcc = uni(0.04, 0.08); tauBrake = uni(0.02, 0.04);
    cplY = 0.05 * uni(0.5, 1.5); cplX = 0.008 * uni(0.5, 1.5) * (getenv("DRIFT") ? atof(getenv("DRIFT")) : 1);   // deg per mm: strafe CCW, forward CW
    gyroScale = 1 + uni(-0.01, 0.01); gyroZero = 30 * gauss();   // hand test: 90 deg read 90.4-90.8
    ticksPerMm = 1.3;
  }
};

struct TracePose { double t, x, y, psi; };
struct TraceEvent { double t; std::string name; int a, b, c, d; };
static bool tracing = false;
static std::vector<TracePose> tPoses;
static std::vector<TraceEvent> tEvents;

class Sim : public rl::Io {
 public:
  Pose p;
  Chassis c;
  double padRg = 1.737;
  bool verbose = false, collided = false;
  int moves = 0;
  long simMs = 0;
  uint16_t lostCount = 0;
  double gyroDeg = 0;     // what the gyro has integrated (deg, true scale)
  double ticksF = 0;
  double u = 0, target = 0, v[3] = {0, 0, 0}, moveGain = 1, wNow[4] = {0, 0, 0, 0};
  int cmdX = 0, cmdY = 0, cmdW = 0;
  enum Mode { OFF, RUN, BRAKE } mode = OFF;
  double motorOnMs = 0;
  Pose atStep[5];
  double stepT[5] = {0, 0, 0, 0, 0};
  bool reachedStep[5] = {false, false, false, false, false};

  void setMix(int8_t x, int8_t y, int8_t w) { cmdX = x; cmdY = y; cmdW = w; }
  // Firmware applyMix: wheel = x -+ y -+ w (FL FR RL RR), u = wheel / peak, |u| < 0.05 off, else
  // 40 + (pwm - 40) |u| (deadzone compensation); each wheel ramps 8 PWM / 10 ms toward it.
  double wheelTarget(int q) const {
    const double wheel[4] = {double(cmdX - cmdY - cmdW), double(cmdX + cmdY + cmdW), double(cmdX + cmdY - cmdW), double(cmdX - cmdY + cmdW)};
    double peak = 0;
    for (double w : wheel) peak = fabs(w) > peak ? fabs(w) : peak;
    if (peak < 1 || target <= 0) return 0;
    double uq = wheel[q] / peak;
    if (fabs(uq) < 0.05) return 0;
    double pw = target > 40 ? 40 + (target - 40) * fabs(uq) : target;
    return uq > 0 ? pw : -pw;
  }
  void step(double dt) {   // dt ms
    simMs += long(dt);
    if (tracing && simMs % 50 == 0) tPoses.push_back({simMs / 1000.0, p.x, p.y, p.psi});
    if (mode == RUN) {
      double sp[4];
      u = 0;
      for (int q = 0; q < 4; ++q) {
        double tq = wheelTarget(q), d = tq - wNow[q];
        wNow[q] += fabs(d) < 0.8 * dt ? d : (d > 0 ? 0.8 * dt : -0.8 * dt);
        double m = fabs(wNow[q]) - c.wheelStall;
        sp[q] = m > 0 ? (wNow[q] > 0 ? 1 : -1) * c.wheelGain * moveGain * m : 0;
        if (fabs(wNow[q]) > u) u = fabs(wNow[q]);
      }
      double vss[3] = {(sp[0] + sp[1] + sp[2] + sp[3]) / 4, c.strafeEff * (-sp[0] + sp[1] + sp[2] - sp[3]) / 4,
                       (-sp[0] + sp[1] - sp[2] + sp[3]) / 4 / c.mmPerDeg};
      for (int k = 0; k < 3; ++k) {
        if (vss[k] == 0 && fabs(v[k]) < 1e-3) v[k] = 0;
        else v[k] += (vss[k] - v[k]) * (1 - exp(-dt / (c.tauAcc * 1000)));
      }
      motorOnMs += dt;
    } else {
      for (int q = 0; q < 4; ++q) wNow[q] = 0;
      for (int k = 0; k < 3; ++k) v[k] *= exp(-dt / ((mode == BRAKE ? c.tauBrake : c.coast[k]) * 1000));
      if (mode == BRAKE) motorOnMs += dt;
    }
    double fwd = v[0] * dt, left = v[1] * dt;
    double dpsi = v[2] * dt + c.cplY * left - c.cplX * fwd;   // deg
    p.x += -fwd * sin(p.psi) - left * cos(p.psi);
    p.y += fwd * cos(p.psi) - left * sin(p.psi);
    p.psi += dpsi * PI / 180;
    if (motorOnMs <= 2500) gyroDeg += dpsi;   // FIFO full: the rest of this stretch is lost
    double ws = fabs(v[0]) * 4 + fabs(v[1]) / c.strafeEff * 4 + fabs(v[2]) * c.mmPerDeg * 4;   // rough: all four wheels
    ticksF += ws * dt * c.ticksPerMm;
    checkCollision();
  }
  void motorsOff() {
    mode = OFF;
    if (motorOnMs > 2500) ++lostCount;
    motorOnMs = 0;
  }
  void run(double ms) { for (double t = 0; t < ms; t += 5) step(5); }
  void checkCollision() {
    for (int i = 0; i < 4; ++i) {
      double f = i < 2 ? FRONT_LEN : -REAR_LEN, l = i % 2 ? HALF_W : -HALF_W;
      double cx = p.x - f * sin(p.psi) - l * cos(p.psi), cy = p.y + f * cos(p.psi) - l * sin(p.psi);
      if (cy >= WALL_Y || (cy >= SIDE_FROM && (cx <= XL || cx >= XR))) collided = true;
    }
  }
  bool measure(int16_t us[rl::NUS], float* rg) override {
    run(100);
    double scans[3][5];
    for (int s = 0; s < 3; ++s) {
      run(120);
      for (int k = 0; k < 5; ++k) scans[s][k] = ping(p, k, 0.02);
    }
    for (int k = 0; k < 5; ++k) {
      double q[3];
      int n = 0;
      for (int s = 0; s < 3; ++s) if (scans[s][k] > 0) q[n++] = scans[s][k];
      double lo = 1e9, hi = -1e9;
      for (int i = 0; i < n; ++i) { lo = q[i] < lo ? q[i] : lo; hi = q[i] > hi ? q[i] : hi; }
      if (n < 2 || hi - lo > 40) { us[k] = -1; continue; }
      us[k] = int16_t((n == 3 ? q[0] + q[1] + q[2] - lo - hi : (q[0] + q[1]) / 2) + 0.5);
    }
    double cx = p.x - 100 * sin(p.psi), cy = p.y + 100 * cos(p.psi);   // colour sensor under the front
    *rg = float(fabs(cx) < 70 && fabs(cy - (N_Y + 100)) < 70 ? padRg : 1.08 + 0.02 * gauss());   // bare floor (field captures 1.07-1.09)
    return moves < 900;
  }
  bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint16_t ms) override {
    ++moves;
    setMix(x, y, w);
    moveGain = 1 + 0.08 * gauss();
    mode = RUN; target = pwm;
    run(ms);
    target = 0;
    while (u > 0) step(5);
    mode = BRAKE; run(60);
    motorsOff();
    run(150);
    return moves < 900;
  }
  bool sweep(int8_t dir, uint8_t pwm, uint16_t ms, float* yaws, int16_t* fronts, uint8_t maxN, uint8_t* n) override {
    ++moves;
    *n = 0;
    setMix(0, 0, dir * 100);
    mode = RUN; target = pwm;
    double t = 0, nextPing = 0;
    auto pingNow = [&]() {
      if (*n >= maxN) return;
      double d = ping(p, rl::FRONT, 0.03);
      float y;
      yaw(&y);
      yaws[*n] = y;
      fronts[*n] = d > 0 ? int16_t(d + 0.5) : -1;
      ++*n;
      nextPing = t + 25 + (d > 0 ? d / 171.5 : 18);
    };
    for (; t < ms; t += 5) { if (t >= nextPing) pingNow(); step(5); }
    target = 0;
    while (u > 0) { step(5); t += 5; }
    mode = BRAKE; run(60);
    motorsOff();
    run(250);
    return moves < 900;
  }
  int drvPolls = 0, drvFocus = 0;
  bool driveStart(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint8_t focus) override {
    ++moves;
    drvPolls = 0;
    drvFocus = focus;
    moveGain = 1 + 0.08 * gauss();
    setMix(x, y, w);
    mode = RUN; target = pwm;
    return moves < 900;
  }
  bool driveSet(int8_t x, int8_t y, int8_t w, uint8_t pwm) override { setMix(x, y, w); target = pwm; return true; }
  bool drivePoll(int16_t us[rl::NUS], float* dt) override {   // main.cpp MegaIo::drivePoll schedules
    static const int OTHERS[4] = {rl::LSIDE, rl::R45, rl::RSIDE, rl::L45};
    static const int SIDES[5] = {rl::LSIDE, rl::RSIDE, rl::LSIDE, rl::RSIDE, rl::FRONT};
    static const int ALL[8] = {rl::FRONT, rl::LSIDE, rl::RSIDE, rl::FRONT, rl::LSIDE, rl::RSIDE, rl::L45, rl::R45};
    static const int LEFT[5] = {rl::FRONT, rl::LSIDE, rl::FRONT, rl::LSIDE, rl::L45};
    static const int RIGHT[5] = {rl::FRONT, rl::RSIDE, rl::FRONT, rl::RSIDE, rl::R45};
    int n = drvPolls++;
    int k = drvFocus == rl::FOCUS_SIDES ? SIDES[n % 5] : drvFocus == rl::FOCUS_ALL ? ALL[n % 8]
          : drvFocus == rl::FOCUS_LEFT ? LEFT[n % 5] : drvFocus == rl::FOCUS_RIGHT ? RIGHT[n % 5]
          : n % 2 ? OTHERS[(n / 2) % 4] : rl::FRONT;
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
    *deg = float(gyroDeg * c.gyroScale + c.gyroZero + 0.05 * gauss());
    return true;
  }
  uint16_t lost() override { return lostCount; }
  uint32_t ticks() override { return uint32_t(ticksF); }
  bool turnTicks(int8_t dir, uint8_t pwm, uint32_t want, uint16_t ms, uint32_t* got) override {
    ++moves;
    double t0 = ticksF;
    setMix(0, 0, dir * 100);
    moveGain = 1 + 0.05 * gauss();
    mode = RUN; target = pwm;
    for (double t = 0; ticksF - t0 < want && t < ms; t += 5) step(5);
    target = 0;   // soft stop: ramp down, brake, settle
    while (u > 0) step(5);
    mode = BRAKE; run(60);
    motorsOff();
    run(250);
    *got = uint32_t(ticksF - t0);
    return moves < 900;
  }
  void event(const char* name, int16_t a, int16_t b, int16_t cc, int16_t d) override {
    if (tracing) tEvents.push_back({simMs / 1000.0, name, a, b, cc, d});
    if (!strcmp(name, "step") && a >= 1 && a <= 4) { atStep[a] = p; reachedStep[a] = true; stepT[a] = simMs / 1000.0; }
    if (verbose) printf("    %-24s %6d %6d %6d %6d   x %5.0f y %5.0f psi %6.1f  t %5.1fs\n", name, a, b, cc, d, p.x, p.y - N_Y, p.psi * 180 / PI, simMs / 1000.0);
  }
};

static void capture(const Pose& p, int16_t ref[rl::NUS], int dropSide) {
  for (int k = 0; k < 5; ++k) {
    double d = senseOne(p, k);
    ref[k] = d > 0 ? int16_t(d + 3 * gauss() + 0.5) : rl::NONE;
  }
  if (dropSide >= 0) ref[dropSide] = rl::NONE;
}
static void makeCal(rl::Calib& cal) {
  for (int k = 0; k < 5; ++k) cal.ref[rl::P][k] = rl::NONE;
  capture({0, N_Y, 0}, cal.ref[rl::N], -1);
  capture({0, TURN_Y, PI / 2}, cal.ref[rl::N_LEFT], rl::LSIDE);
  capture({BOX_LX, TURN_Y, PI / 2}, cal.ref[rl::BOX_LEFT], rl::LSIDE);
  capture({0, TURN_Y, -PI / 2}, cal.ref[rl::N_RIGHT], rl::RSIDE);
  capture({BOX_RX, TURN_Y, -PI / 2}, cal.ref[rl::BOX_RIGHT], rl::RSIDE);
  cal.greenMilli = 857;
  cal.redMilli = 1737;
}

struct Result { double st[5]; bool ok, good, collided, left, wantLeft; double ex, ey, epsi, t; const char* why; Pose start, atN; bool reachedN; };
static Result runOne(double arcDeg, double r, double aimErr, bool green, bool verbose) {
  rl::Calib cal;
  makeCal(cal);
  Sim sim;
  sim.c.draw();
  MOUNT[rl::FRONT].cone = uni(7, getenv("CONE_MAX") ? atof(getenv("CONE_MAX")) : 15);   // the real beam width is not known: drawn per run
  sim.verbose = verbose;
  // the start arc's centre: ARC_CY mm out from the main wall (video: the outer semicircle ~210 mm out, ~1 m radius)
  static const double arcCy = getenv("ARC_CY") ? atof(getenv("ARC_CY")) : 210;
  double a = arcDeg * PI / 180, px = r * sin(a), py = WALL_Y - arcCy - r * cos(a);   // + arc = right of the light
  double toN = atan2(-(0 - px), N_Y - py);
  sim.p = {px, py, toN + aimErr * PI / 180};
  sim.padRg = green ? 0.857 : 1.737;
  Result res;
  res.start = sim.p;
  rl::Runner runner(sim, cal);
  res.ok = runner.run();
  res.left = runner.turn() > 0;
  res.wantLeft = green;
  double tx = res.left ? BOX_LX : BOX_RX, tpsi = res.left ? 90 : -90;
  res.ex = sim.p.x - tx; res.ey = sim.p.y - TURN_Y; res.epsi = sim.p.psi * 180 / PI - tpsi;
  res.collided = sim.collided;
  res.good = res.ok && res.left == res.wantLeft && fabs(res.ex) < 30 && fabs(res.ey) < 30 && fabs(res.epsi) < 4 && !sim.collided;
  res.t = sim.simMs / 1000.0;
  for (int k = 0; k < 5; ++k) res.st[k] = sim.stepT[k];
  res.reachedN = sim.reachedStep[3];
  res.atN = sim.atStep[3];
  res.why = !res.ok ? runner.reason() : sim.collided ? "touched_wall" : res.left != res.wantLeft ? "wrong_box"
          : res.good ? "OK" : fabs(res.epsi) >= 4 ? "parked_heading_off" : "parked_position_off";
  return res;
}

static int traceScenarios(const char* path);
static const double ARCS[13] = {-45, -37.5, -30, -22.5, -15, -7.5, 0, 7.5, 15, 22.5, 30, 37.5, 45};

int main(int argc, char** argv) {
  if (getenv("TRACE_OUT")) return traceScenarios(getenv("TRACE_OUT"));
  if (getenv("CALSHOW")) {   // the simulated captures next to the field's (field-2026-11-03a)
    srand(1);
    rl::Calib cal;
    makeCal(cal);
    const char* names[6] = {"P", "N", "N_LEFT", "BOX_LEFT", "N_RIGHT", "BOX_RIGHT"};
    for (int i = 1; i < 6; ++i) printf("%-10s L45 %5d R45 %5d F %5d LS %5d RS %5d\n", names[i], cal.ref[i][0], cal.ref[i][1], cal.ref[i][2], cal.ref[i][3], cal.ref[i][4]);
    return 0;
  }
  int seeds = getenv("SEEDS") ? atoi(getenv("SEEDS")) : 4;
  int seed0 = getenv("SEED") ? atoi(getenv("SEED")) : 3848;
  int verboseRun = argc > 1 ? atoi(argv[1]) : -1;
  int passed = 0, runs = 0, nReasons = 0, reasonCount[32];
  const char* reasons[32];
  double sumT = 0, maxT = 0, sumAbs[3] = {0, 0, 0}, sumNx = 0, sumNpsi = 0;
  int nN = 0;
  double ph[4] = {0, 0, 0, 0};
  std::vector<double> times;
  for (int s = 0; s < seeds; ++s) {
    srand(seed0 + s);
    for (int ai = 0; ai < 13; ++ai)
      for (int col = 0; col < 2; ++col) {
        srand(seed0 * 7919 + s * 131 + ai * 2 + col);   // same start, robot and noise per run whatever the code does
        double r = uni(1250, 1350), aim = uni(-4, 4);
        bool v = runs == verboseRun;
        Result res = runOne(ARCS[ai], r, aim, col == 0, v);
        ++runs;
        passed += res.good;
        sumT += res.t;
        if (res.st[4] > 0) { ph[0] += res.st[2]; ph[1] += res.st[3] - res.st[2]; ph[2] += res.st[4] - res.st[3]; ph[3] += res.t - res.st[4]; }
        maxT = res.t > maxT ? res.t : maxT;
        if (res.ok && res.left == res.wantLeft) { sumAbs[0] += fabs(res.ex); sumAbs[1] += fabs(res.ey); sumAbs[2] += fabs(res.epsi); }
        if (res.reachedN) { ++nN; sumNx += fabs(res.atN.x); sumNpsi += fabs(res.atN.psi * 180 / PI); }
        int j = 0;
        while (j < nReasons && strcmp(reasons[j], res.why)) ++j;
        if (j == nReasons) { reasons[nReasons] = res.why; reasonCount[nReasons++] = 0; }
        ++reasonCount[j];
        if (getenv("LIST") || v)
          printf("%3d arc %+5.1f %s | %-22s box err x %+5.0f y %+5.0f psi %+5.1f | at N x %+4.0f psi %+5.1f | %5.1fs %s\n", runs - 1, ARCS[ai],
                 col == 0 ? "green" : "red  ", res.why, res.ex, res.ey, res.epsi, res.atN.x, res.atN.psi * 180 / PI, res.t, res.good ? "OK" : "FAIL");
      }
  }
  printf("outcomes:");
  for (int j = 0; j < nReasons; ++j) printf("  %s %d", reasons[j], reasonCount[j]);
  printf("\nmean |err| at the box: x %.0f mm  y %.0f mm  heading %.1f deg;  at N before the colour: |x| %.0f mm |heading| %.1f deg\n",
         sumAbs[0] / runs, sumAbs[1] / runs, sumAbs[2] / runs, nN ? sumNx / nN : 0, nN ? sumNpsi / nN : 0);
  printf("mean run time %.1f s (max %.1f s): start to N's line %.1f, settle on N %.1f, colour + turn %.1f, to the box %.1f\n", sumT / runs, maxT,
         ph[0] / runs, ph[1] / runs, ph[2] / runs, ph[3] / runs);
  printf("%d/%d parked within 30 mm and 4 deg, no wall touched (%.1f %%)\n", passed, runs, 100.0 * passed / runs);
  return 0;
}

// Viewer: every start on the arc (both colours) traced to one JSON file.
static int traceScenarios(const char* path) {
  srand(getenv("SEED") ? atoi(getenv("SEED")) : 2026);
  FILE* f = fopen(path, "w");
  if (!f) return 1;
  fprintf(f, "{\"arcCy\":%g,\"wallY\":%g,\"xl\":%g,\"xr\":%g,\"sideFrom\":%g,\"nY\":%g,\"boxY\":%g,\"boxL\":%g,\"boxR\":%g,\"runs\":[", getenv("ARC_CY") ? atof(getenv("ARC_CY")) : 210.0, WALL_Y, XL, XR, SIDE_FROM, N_Y, TURN_Y, BOX_LX, BOX_RX);
  int run = 0;
  for (int ai = 0; ai < 13; ++ai)
    for (int col = 0; col < 2; ++col, ++run) {
      srand((getenv("SEED") ? atoi(getenv("SEED")) : 2026) * 7919 + run);
      double r = uni(1250, 1350), aim = uni(-4, 4);
      tracing = true;
      tPoses.clear();
      tEvents.clear();
      Result res = runOne(ARCS[ai], r, aim, col == 0, false);
      tracing = false;
      fprintf(f, "%s{\"label\":\"arc %+.1f deg, %s\",\"colour\":\"%s\",\"ok\":%s,\"good\":%s,\"reason\":\"%s\",\"collided\":%s,\"err\":[%.0f,%.0f,%.1f],\"poses\":[",
              run ? "," : "", ARCS[ai], col == 0 ? "green" : "red", col == 0 ? "green" : "red", res.ok ? "true" : "false", res.good ? "true" : "false",
              res.why, res.collided ? "true" : "false", res.ex, res.ey, res.epsi);
      for (size_t i = 0; i < tPoses.size(); ++i) fprintf(f, "%s[%.2f,%.0f,%.0f,%.3f]", i ? "," : "", tPoses[i].t, tPoses[i].x, tPoses[i].y, tPoses[i].psi);
      fprintf(f, "],\"events\":[");
      for (size_t i = 0; i < tEvents.size(); ++i)
        fprintf(f, "%s[%.2f,\"%s\",%d,%d,%d,%d]", i ? "," : "", tEvents[i].t, tEvents[i].name.c_str(), tEvents[i].a, tEvents[i].b, tEvents[i].c, tEvents[i].d);
      fprintf(f, "]}");
      fprintf(stderr, "run %2d arc %+5.1f %-5s %-22s %5.1fs\n", run, ARCS[ai], col == 0 ? "green" : "red", res.why, res.t);
    }
  fprintf(f, "]}\n");
  fclose(f);
  return 0;
}

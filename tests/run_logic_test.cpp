// Host test of the on-Mega tournament controller (firmware/hardware_probe_mega/src/run_logic.*)
// in a simulated arena. Geometry is an approximation of the 30 Oct practice field and the robot
// model carries the quirks seen there (soft-start dead time, spread in move length, strafes
// turning CCW, forward drives CW, noisy and missing echoes, a wide beam cone). It checks the
// logic, not the field: real accuracy needs field runs.
// Build/run: tools/verify_run_logic.sh
#include "../firmware/hardware_probe_mega/src/run_logic.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double WALL_Y = 1154;               // main wall, P at (0, 0) facing +y
static const double SIDE_X = 770, SIDE_FROM = WALL_Y - 480;   // short side walls at +/-770, 48 cm long
static const double N_Y = WALL_Y - 418;          // N: the front reads 268 there
static const double PI = 3.14159265358979;
struct Mount { double fwd, left, deg; };
static const Mount MOUNT[5] = {{150, 95, 45}, {150, -95, -45}, {150, 0, 0}, {0, 115, 90}, {0, -115, -90}};

static double urand() { return rand() / (double)RAND_MAX; }
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

static void sense(const Pose& p, double out[5], bool noisy) {
  for (int k = 0; k < 5; ++k) {
    const Mount& m = MOUNT[k];
    double sx = p.x - m.fwd * sin(p.psi) - m.left * cos(p.psi);
    double sy = p.y + m.fwd * cos(p.psi) - m.left * sin(p.psi);
    double best = -1;
    for (double c = -12; c <= 12.01; c += 3) {   // beam cone
      double d = ray(sx, sy, p.psi + (m.deg + c) * PI / 180);
      if (d > 0 && (best < 0 || d < best)) best = d;
    }
    if (best > 2500 || best < 20) best = -1;
    if (noisy && best > 0) {
      if (urand() < 0.05) best = -1;
      else best += 4 * gauss();
    }
    out[k] = best;
  }
}

class Sim : public rl::Io {
 public:
  Pose p;
  double padRg = 1.66;   // colour under the sensor at N
  bool verbose = false, outOfSector = false;
  int pulses = 0;
  double yawBias = 0;
  bool measure(int16_t us[rl::NUS], float* rg) override {
    double scans[3][5];
    for (int s = 0; s < 3; ++s) sense(p, scans[s], true);
    for (int k = 0; k < 5; ++k) {
      double v[3];
      int n = 0;
      for (int s = 0; s < 3; ++s) if (scans[s][k] > 0) v[n++] = scans[s][k];
      double lo = 1e9, hi = -1e9;
      for (int i = 0; i < n; ++i) { lo = v[i] < lo ? v[i] : lo; hi = v[i] > hi ? v[i] : hi; }
      if (n < 2 || hi - lo > 40) { us[k] = -1; continue; }
      double med = n == 3 ? v[0] + v[1] + v[2] - lo - hi : (v[0] + v[1]) / 2;
      us[k] = int16_t(med + 0.5);
    }
    double cx = p.x - 150 * sin(p.psi), cy = p.y + 150 * cos(p.psi);   // colour sensor, front
    *rg = float(hypot(cx - 0, cy - (N_Y + 150)) < 60 ? padRg : 1.05);
    return true;
  }
  bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint16_t ms) override {
    ++pulses;
    double fl = x - y - w, fr = x + y + w, rl = x + y - w, rr = x - y + w, peak = 1;
    const double wheel[4] = {fl, fr, rl, rr};
    for (double v : wheel) peak = fabs(v) > peak ? fabs(v) : peak;
    double ux = x / peak, uy = y / peak, uw = w / peak;
    double te = ms > 60 ? (ms - 60) : 0, spread = 1 + 0.15 * gauss();
    double vx = 0.6 * pwm / 85, vy = 0.35 * pwm / 100, vw = 0.25 * pwm / 120;   // per ms
    if (pwm * fabs(ux) < 45) ux = 0;   // static friction
    if (pwm * fabs(uy) < 60) uy = 0;
    if (pwm * fabs(uw) < 50) uw = 0;
    double fwd = vx * ux * te * spread, left = vy * uy * te * spread;
    double turn = vw * uw * te * spread + (uy ? 0.025 * te : 0) - (ux ? 0.008 * te : 0);
    p.x += -fwd * sin(p.psi) - left * cos(p.psi);
    p.y += fwd * cos(p.psi) - left * sin(p.psi);
    p.psi += turn * PI / 180;
    if (hypot(p.x, p.y - WALL_Y) > WALL_Y + 400 || p.y > WALL_Y - 100) outOfSector = true;
    return pulses < 600;
  }
  // Continuous drive: 25 ms steps; speed 0.6 mm/ms * (pwm-45)/40 (stalls below 45), a forward drive turns the
  // robot slowly CW, one front ping per step (4 mm noise, 5 % no echo, 3 % spurious far echo), 5 mm of coast.
  double drvX = 0, drvY = 0, drvW = 0;
  int drvPwm = 0;
  double speed() const { return drvPwm < 45 ? 0 : 0.6 * (drvPwm - 45) / 40; }
  bool driveStart(int8_t x, int8_t y, int8_t w, uint8_t pwm) override { ++pulses; return driveSet(x, y, w, pwm); }
  bool driveSet(int8_t x, int8_t y, int8_t w, uint8_t pwm) override { drvX = x / 100.0; drvY = y / 100.0; drvW = w / 100.0; drvPwm = pwm; return true; }
  bool drivePoll(int16_t* front, float* dt) override {
    const double ms = 25, v = speed() * (1 + 0.05 * gauss());
    p.x += -(drvX * v * ms) * sin(p.psi) - (drvY * 0.5 * v * ms) * cos(p.psi);
    p.y += (drvX * v * ms) * cos(p.psi) - (drvY * 0.5 * v * ms) * sin(p.psi);
    p.psi += ((drvW * 0.25 - 0.004 * drvX) * ms) * PI / 180;
    if (hypot(p.x, p.y - WALL_Y) > WALL_Y + 400 || p.y > WALL_Y - 100) outOfSector = true;
    double r[5];
    sense(p, r, true);
    double f = r[2];
    if (f > 0 && urand() < 0.03) f = 100 + 400 * urand();   // spurious echo
    *front = f > 0 ? int16_t(f + 0.5) : -1;
    *dt = float(ms / 1000);
    return pulses < 600;
  }
  bool driveEnd() override {
    const double coast = speed() * 1000 * 0.02;   // mm
    p.x += -(drvX * coast) * sin(p.psi);
    p.y += (drvX * coast) * cos(p.psi);
    drvPwm = 0;
    if (p.y > WALL_Y - 100) outOfSector = true;
    return true;
  }
  bool yaw(float* deg) override { *deg = float(p.psi * 180 / PI + yawBias); return true; }
  uint16_t lost() override { return 0; }
  void event(const char* name, int16_t a, int16_t b, int16_t c, int16_t d) override {
    if (verbose) printf("    %-28s %6d %6d %6d %6d   pose x %5.0f y %5.0f psi %6.1f\n", name, a, b, c, d, p.x, p.y, p.psi * 180 / PI);
  }
};

static void capture(const Pose& p, int16_t ref[rl::NUS], int dropSide) {
  double v[5];
  sense(p, v, false);
  for (int k = 0; k < 5; ++k) ref[k] = v[k] > 0 ? int16_t(v[k] + 0.5) : rl::NONE;
  if (dropSide >= 0) ref[dropSide] = rl::NONE;   // start-facing side: no field wall (upload rule)
}

int main(int argc, char** argv) {
  bool verbose = argc > 1 && argv[1][0] == 'v';
  srand(3848);
  const double BOX_X = SIDE_X - 150 - 47;   // parked facing the side wall, front reading 47
  rl::Calib cal;
  capture({0, 0, 0}, cal.ref[rl::P], -1);
  capture({0, N_Y, 0}, cal.ref[rl::N], -1);
  capture({0, N_Y, PI / 2}, cal.ref[rl::N_LEFT], rl::LSIDE);
  capture({-BOX_X, N_Y, PI / 2}, cal.ref[rl::BOX_LEFT], rl::LSIDE);
  capture({0, N_Y, -PI / 2}, cal.ref[rl::N_RIGHT], rl::RSIDE);
  capture({BOX_X, N_Y, -PI / 2}, cal.ref[rl::BOX_RIGHT], rl::RSIDE);
  cal.greenMilli = 830;
  cal.redMilli = 1660;
  int passed = 0, runs = 0;
  double arc = argc > 2 ? atof(argv[2]) : 20, aim = argc > 3 ? atof(argv[3]) : 15;
  for (int i = 0; i < 24; ++i) {
    // Start on the arc ~1.2 m from the light within +/-arc deg (field-tested starts were ~30-40 cm
    // either side, about +/-20 deg), pointing at the wall within +/-aim deg.
    double a = (-arc + 2 * arc * urand()) * PI / 180, r = WALL_Y + 100 * urand();
    Sim sim;
    sim.verbose = verbose && i == (argc > 4 ? atoi(argv[4]) : 0);
    sim.p = {-r * sin(a), WALL_Y - r * cos(a), a + (-aim + 2 * aim * urand()) * PI / 180};
    sim.padRg = i % 2 ? 0.83 : 1.66;
    sim.yawBias = 30 * gauss();   // the gyro's zero is wherever it was placed
    Pose start = sim.p;
    rl::Runner runner(sim, cal);
    bool ok = runner.run();
    bool left = runner.turn() > 0;
    double tx = left ? -BOX_X : BOX_X, tpsi = left ? 90 : -90;
    double ex = sim.p.x - tx, ey = sim.p.y - N_Y, epsi = sim.p.psi * 180 / PI - tpsi;
    bool wantLeft = sim.padRg < 1.2;
    bool good = ok && left == wantLeft && fabs(ex) < 30 && fabs(ey) < 30 && fabs(epsi) < 4 && !sim.outOfSector;
    ++runs;
    passed += good;
    printf("start x %5.0f y %5.0f psi %+5.1f | %s %s | box err x %+5.0f y %+5.0f psi %+5.1f | %3d pulses %s%s\n",
           start.x, start.y, start.psi * 180 / PI, wantLeft ? "green" : "red  ", ok ? "done   " : runner.reason(),
           ex, ey, epsi, sim.pulses, good ? "OK" : "FAIL", sim.outOfSector ? " (left the sector)" : "");
  }
  printf("%d/%d simulated runs parked within 30 mm and 4 deg\n", passed, runs);
  return passed == runs ? 0 : 1;
}

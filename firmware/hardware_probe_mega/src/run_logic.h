// Tournament run on the Mega: stop-and-go controller ported from tools/seek_steps.py (field-tested
// 30 Oct). Positions come from the ultrasonics, heading from the gyro read only while stopped.
// Pure logic: all I/O goes through rl::Io, so the same code runs on the Mega (main.cpp) and on a
// host against a simulated arena (tests/run_logic_test.cpp).
#pragma once
#include <math.h>
#include <stdint.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RL_F(s) PSTR(s)   // event names and reasons live in flash on the Mega
#else
#define RL_F(s) (s)
#endif

namespace rl {

enum { L45 = 0, R45 = 1, FRONT = 2, LSIDE = 3, RSIDE = 4, NUS = 5 };   // ultrasonic_mm order
// P is no longer used by the run (deleted 2 Nov: the start goes straight to N); its slot stays so the
// stored calibration layout (EEPROM, CAL 0..5) does not change. Upload sends it with no targets.
enum Pose { P = 0, N = 1, N_LEFT = 2, BOX_LEFT = 3, N_RIGHT = 4, BOX_RIGHT = 5, NPOSE = 6 };
const int16_t NONE = 9999;   // the capture had no steady reading for this sensor
const int16_t NOT_PINGED = -2;   // drivePoll: this sensor was not pinged on this poll
enum Focus { FOCUS_FRONT = 0, FOCUS_SIDES = 1, FOCUS_ALL = 2, FOCUS_LEFT = 3, FOCUS_RIGHT = 4 };   // drivePoll ping schedule

// Recorded readings of each checkpoint (by ultrasonic index; NONE = not a target) and the colour
// references (red/green count ratio x 1000).
struct Calib {
  int16_t ref[NPOSE][NUS];
  uint16_t greenMilli, redMilli;
};

// Distance tracker for one ultrasonic during forward moves (all five have one): constant-velocity
// Kalman filter, state (distance mm, speed mm/s; negative = approaching). predict() every poll,
// update() with each echo. An echo far from the prediction (a spurious reflection) is rejected by
// the innovation gate; a missing echo just means prediction continues. See Runner::driveForward.
class Kalman1D {
 public:
  void init(float d0, float sigmaZ = 6.0f) {
    d = d0; v = 0; r = sigmaZ * sigmaZ;
    p00 = r; p01 = 0; p11 = 150.0f * 150.0f;
  }
  void predict(float dt, float accel = 500.0f) {   // accel: mm/s^2 the speed may change by (1 sigma)
    float q = accel * accel;
    d += v * dt;
    p00 += dt * (2 * p01 + dt * p11) + q * dt * dt * dt * dt / 4;
    p01 += dt * p11 + q * dt * dt * dt / 2;
    p11 += q * dt * dt;
  }
  // false = rejected (outside max(30 mm, 3.5 sigma) of the prediction)
  bool update(float z) {
    float s = p00 + r, y = z - d, gate = 3.5f * sqrtf(s);
    if (gate < 30.0f) gate = 30.0f;
    if (y > gate || y < -gate) return false;
    float k0 = p00 / s, k1 = p01 / s;
    d += k0 * y;
    v += k1 * y;
    float n00 = (1 - k0) * p00, n01 = (1 - k0) * p01, n11 = p11 - k1 * p01;
    p00 = n00; p01 = n01; p11 = n11;
    return true;
  }
  float dist() const { return d; }
  float speed() const { return v; }
 private:
  float d, v, r, p00, p01, p11;
};

// How long a timed move must run to cover `amount` (mm or deg): amount ~ a * ms + b, a least-squares
// fit over this run's own moves with forgetting (battery and floor change it), so a small error gets a
// small move. Until two clearly different lengths have been tried, the prior rate and dead time.
struct MoveModel {
  float s0, sx, sy, sxx, sxy, rate, dead;
  void init(float rate0, float dead0) { s0 = sx = sy = sxx = sxy = 0; rate = rate0; dead = dead0; }
  float msFor(float amount) const {
    if (s0 > 1.5f) {
      float den = s0 * sxx - sx * sx;   // s0^2 * variance of the lengths tried
      if (den > 225.0f * s0 * s0) {     // lengths spread by more than ~15 ms
        float a = (s0 * sxy - sx * sy) / den, b = (sy - a * sx) / s0;
        if (a > 0.2f * rate) return (amount - b) / a;
      }
    }
    return dead + amount / rate;
  }
  void learn(float ms, float amount) {
    const float keep = 0.7f;
    s0 = keep * s0 + 1; sx = keep * sx + ms; sy = keep * sy + amount;
    sxx = keep * sxx + ms * ms; sxy = keep * sxy + ms * amount;
    if (ms > dead + 15 && amount > 0) {
      float r = amount / (ms - dead);
      rate = 0.5f * rate + 0.5f * (r < 0.25f * rate ? 0.25f * rate : r > 4 * rate ? 4 * rate : r);
    }
  }
};

class Io {
 public:
  // Fresh readings once the robot is still: per sensor the median of 3 scans, -1 if fewer than
  // two echoes or they varied by more than 40 mm; *rg = red/green ratio (0 if unknown).
  // false = stop requested or the robot cannot measure: the run ends.
  virtual bool measure(int16_t us[NUS], float* rg) = 0;
  // One timed mecanum move (x forward, y left, w CCW, -100..100): soft start to pwm, hold until ms,
  // ramp down at the soft-start rate, short-brake ~60 ms (no coasting), settle. The gyro angle is
  // current when it returns.
  virtual bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint16_t ms) = 0;
  // One smooth continuous rotation (dir +1 CCW) at pwm for ms (soft start, ramp down, brake), the
  // front sensor pinged every ~25 ms on the way. The gyro is not read while turning (motor noise on
  // its bus); its buffered samples give each ping's heading once stopped. Fills up to maxN pairs
  // (yaw deg, front mm or -1); false = stopped or the gyro lost data.
  virtual bool sweep(int8_t dir, uint8_t pwm, uint16_t ms, float* yaws, int16_t* fronts, uint8_t maxN, uint8_t* n) = 0;
  // CCW-positive heading in degrees from the last measurement; false if the gyro is not zeroed.
  virtual bool yaw(float* deg) = 0;
  // Gyro data losses so far (overflows, restarts, resets): a change means the angle jumped.
  virtual uint16_t lost() = 0;
  // Continuous forward motion (Runner::driveForward): driveStart turns the motors on, drivePoll waits
  // until >= 25 ms after the previous poll, services the robot (STOP, watchdog) and pings one sensor:
  // the front on every other poll, the other four in turn between (LSIDE, R45, RSIDE, L45), so the
  // front updates at ~20 Hz and each other sensor at ~5 Hz. us[k] = mm or -1 (no echo) for the
  // pinged sensor, NOT_PINGED for the rest; *dt = seconds since the previous poll, or since
  // driveStart. driveSet changes the speed without stopping; driveEnd stops, lets the robot settle
  // and returns. false from any of them = stopped (STOP / button): the run ends.
  // focus FOCUS_SIDES (strafes): LSIDE, RSIDE, LSIDE, RSIDE, FRONT in turn instead (sides ~16 Hz).
  // focus FOCUS_ALL (combined moves at N): FRONT, LSIDE, RSIDE, FRONT, LSIDE, RSIDE, L45, R45 in turn;
  // FOCUS_LEFT / FOCUS_RIGHT (after a turn): FRONT, side, FRONT, side, 45 deg on that side.
  virtual bool driveStart(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint8_t focus) = 0;
  virtual bool driveSet(int8_t x, int8_t y, int8_t w, uint8_t pwm) = 0;
  virtual bool drivePoll(int16_t us[NUS], float* dt) = 0;
  virtual bool driveEnd() = 0;
  // Progress report: name (RL_F string) and up to four integers.
  virtual void event(const char* name, int16_t a = 0, int16_t b = 0, int16_t c = 0, int16_t d = 0) = 0;
  virtual ~Io() {}
};

class Runner {
 public:
  Runner(Io& io, const Calib& cal);
  // Steps first..last (1 walk + sweep, 2 settle on N, 3 colour + turn + check, 4 park; see run()). forceTurn +1/-1 picks the box when
  // step 3 is skipped. false: stopped; reason() says why (RL_F string).
  bool run(uint8_t first = 1, uint8_t last = 4, int8_t forceTurn = 0);
  // Test of the Kalman-tracked slow approach alone: drive straight at what is ahead and stop `target` mm short.
  bool creepTest(int16_t target);
  const char* reason() const { return why; }
  int8_t turn() const { return turnDir; }

 private:
  Io& io;
  const Calib& cal;
  const char* why;
  MoveModel back, side;   // timed reverse and strafe moves, mm (learned)
  MoveModel rot[2];       // rotation, deg, at rotPwm[0] (coarse) and rotPwm[1] (fine) (learned)
  uint8_t rotPwm[2];      // raised if the robot does not turn at all (unknown breakaway PWM)
  uint16_t rotMinMs[2];   // after a pulse that did not turn: the next one is longer
  float sweepRate;        // deg/ms of a sweep at rotPwm[1] (learned from each sweep)
  int8_t wcomp[2][2];  // rotation mixed into a move [axis][sign > 0] to cancel its spin (learned)
  float goal;          // gyro heading to hold
  bool goalSet;
  uint16_t goalLost;
  int8_t turnDir;      // +1 left (green), -1 right (red)

  bool fail(const char* r) { why = r; return false; }
  bool measure(int16_t us[NUS], float* rg = 0);
  bool yawNow(float* y) { return io.yaw(y); }
  bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, float ms, uint16_t* used = 0, uint16_t lo = 50, uint16_t hi = 300);
  bool errors(Pose pose, const int16_t us[NUS], bool creep, float* ex, bool* hasEx, float* ey, bool* hasEy);
  bool translate(uint8_t axis, float err, int16_t us[NUS], float maxMs = 300, uint16_t* used = 0);
  float stopTau;       // seconds the robot keeps moving after the stop decision (learned from each drive)
  bool driveForward(float stopAt, float maxMs, uint8_t pwm, int16_t us[NUS]);
  bool learn(uint8_t axis, bool hasBefore, float before, bool hasAfter, float after, uint16_t ms);
  bool hold(int16_t us[NUS]);
  bool rotateTo(float target, float tol, bool strict, float* reached = 0);
  bool rotatePulse(int8_t dir, float y0, int16_t us[NUS], float* y1, bool* turned);
  bool seek(Pose pose, bool creep, float tol, int16_t us[NUS]);
  bool settle(Pose pose, uint8_t moves, int16_t us[NUS], float target = 88, float dead = 15);
  bool turnOnce(bool ccw);
  bool squareToSides();
  float fixRate[3], fixRate0[3];
  float cornerMm;
  float accuracySet(Pose pose, uint8_t set, const int16_t us[NUS]) const;
  bool measureFine(int16_t us[NUS], bool twice);
  bool fixPose(Pose pose, uint8_t set, float target, uint8_t maxMoves, int16_t us[NUS]);
  bool walkTo(int16_t target, int16_t us[NUS]);
  float stopTauY;      // seconds a strafe keeps going after the stop decision (learned)
  bool driveSide(Pose pose, float frac, int16_t us[NUS], bool* used);
  float accuracy(Pose pose, const int16_t us[NUS]) const;
  bool squareToBeams(Pose pose, int16_t us[NUS]);
  bool descend(Pose pose, bool turns, int16_t us[NUS], bool mustReach = true);
  bool approach(int16_t us[NUS]);
  bool squareHere(float half = 70);
  bool sweepTo(float end, float* yaws, int16_t* fronts, uint8_t* n, uint8_t maxN);
  bool turnTo(bool ccw);
  bool park(Pose pose, int16_t us[NUS]);
  bool setGoal(float g);
  bool faceWall(int16_t us[NUS]);
  bool driveTo(Pose pose, uint8_t set, float tolX, float tolY, int16_t us[NUS]);
  bool align(Pose pose, uint8_t set, float target, float deadX, float deadY, uint8_t maxMoves, int16_t us[NUS]);
  bool straighten(uint8_t set, const int16_t a[NUS], int16_t us[NUS]);
  bool squareByMove(uint8_t set, float dist, int16_t us[NUS]);
  int16_t snapSum;     // driveForward: side-wall spacing (L + R) to snapshot at, 0 = off
  bool snapped;        // driveForward saw both side walls at that spacing: snap holds front / sides then
  int16_t snap[NUS];
  float beamSlope[3];  // mm of the stage's 45 deg reading per deg CCW, learned from each straighten turn (0 = unknown)
};

}  // namespace rl

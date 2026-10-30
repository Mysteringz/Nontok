// Tournament run on the Mega: stop-and-go controller ported from tools/seek_steps.py (field-tested
// 30 Oct). Positions come from the ultrasonics, heading from the gyro read only while stopped.
// Pure logic: all I/O goes through rl::Io, so the same code runs on the Mega (main.cpp) and on a
// host against a simulated arena (tests/run_logic_test.cpp).
#pragma once
#include <stdint.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RL_F(s) PSTR(s)   // event names and reasons live in flash on the Mega
#else
#define RL_F(s) (s)
#endif

namespace rl {

enum { L45 = 0, R45 = 1, FRONT = 2, LSIDE = 3, RSIDE = 4, NUS = 5 };   // ultrasonic_mm order
enum Pose { P = 0, N = 1, N_LEFT = 2, BOX_LEFT = 3, N_RIGHT = 4, BOX_RIGHT = 5, NPOSE = 6 };
const int16_t NONE = 9999;   // the capture had no steady reading for this sensor

// Recorded readings of each checkpoint (by ultrasonic index; NONE = not a target) and the colour
// references (red/green count ratio x 1000).
struct Calib {
  int16_t ref[NPOSE][NUS];
  uint16_t greenMilli, redMilli;
};

class Io {
 public:
  // Fresh readings once the robot is still: per sensor the median of 3 scans, -1 if fewer than
  // two echoes or they varied by more than 40 mm; *rg = red/green ratio (0 if unknown).
  // false = stop requested or the robot cannot measure: the run ends.
  virtual bool measure(int16_t us[NUS], float* rg) = 0;
  // One timed mecanum move (x forward, y left, w CCW, -100..100) at pwm for ms, then settle.
  virtual bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, uint16_t ms) = 0;
  // CCW-positive heading in degrees from the last measurement; false if the gyro is not zeroed.
  virtual bool yaw(float* deg) = 0;
  // Gyro data losses so far (overflows, restarts, resets): a change means the angle jumped.
  virtual uint16_t lost() = 0;
  // Progress report: name (RL_F string) and up to four integers.
  virtual void event(const char* name, int16_t a = 0, int16_t b = 0, int16_t c = 0, int16_t d = 0) = 0;
  virtual ~Io() {}
};

class Runner {
 public:
  Runner(Io& io, const Calib& cal);
  // Steps first..last (1 approach + square to the wall, 2 match N, 3 colour turn, 4 park). forceTurn +1/-1 picks the box when
  // step 3 is skipped. false: stopped; reason() says why (RL_F string).
  bool run(uint8_t first = 1, uint8_t last = 4, int8_t forceTurn = 0);
  const char* reason() const { return why; }
  int8_t turn() const { return turnDir; }

 private:
  Io& io;
  const Calib& cal;
  const char* why;
  float k[2];          // mm of error removed per ms of move, x and y (learned)
  int8_t wcomp[2][2];  // rotation mixed into a move [axis][sign > 0] to cancel its spin (learned)
  float rate[2];       // deg per ms beyond the dead time, at ROT_PWM and ROT_FINE_PWM (learned)
  float goal;          // gyro heading to hold
  bool goalSet;
  uint16_t goalLost;
  int8_t turnDir;      // +1 left (green), -1 right (red)

  bool fail(const char* r) { why = r; return false; }
  bool measure(int16_t us[NUS], float* rg = 0);
  bool yawNow(float* y) { return io.yaw(y); }
  bool pulse(int8_t x, int8_t y, int8_t w, uint8_t pwm, float ms, uint16_t* used = 0);
  bool errors(Pose pose, const int16_t us[NUS], bool creep, float* ex, bool* hasEx, float* ey, bool* hasEy);
  bool translate(uint8_t axis, float err, int16_t us[NUS], float maxMs = 300, uint16_t* used = 0);
  bool learn(uint8_t axis, bool hasBefore, float before, bool hasAfter, float after, uint16_t ms);
  bool hold(int16_t us[NUS]);
  bool rotateTo(float target, float tol, bool strict, float* reached = 0);
  bool rotatePulse(int8_t dir, float y0, int16_t us[NUS], float* y1, bool* turned);
  bool seek(Pose pose, bool creep, float tol, int16_t us[NUS]);
  bool squareToBeams(Pose pose, int16_t us[NUS]);
  bool descend(Pose pose, bool turns, int16_t us[NUS]);
  bool approach(int16_t mm, int16_t us[NUS]);
  bool squareHere();
  bool turnTo(bool ccw);
  bool park(Pose pose, int16_t us[NUS]);
  bool setGoal(float g);
};

}  // namespace rl

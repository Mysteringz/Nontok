// stepper_driver.h - DRV8825 / A4988 stepper on the "42" expansion board, header only.
//
// Wiring (shield PL header, verified 21 Nov):
//   STEP = PL3 (D46)   DIR = PL4 (D45)   EN = PL5 (D44, LOW = driver on)   G = shield GND
//   Motor power: 12 V in parallel with the shield's 12 V input.
//   DIP switches at 1/16 microstepping: 200 full steps x 16 = 3200 steps per turn.
//   D46 is also Timer 5's OC5A output; nothing here uses Timer 5, so it stays free.
//
// Two ways to move:
//   Blocking:      stepper.moveDegrees(90);           returns when done (or when abort() says stop)
//   Non-blocking:  stepper.startDegrees(90);           then call stepper.service() often from loop();
//                  it returns true while still moving. Steps are timed with micros(), so a caller
//                  that is busy for longer than one step period just makes the move slower.
//
// Moves ramp the step rate up and down (rampSteps) so the motor does not skip at speed.
// The blocking move calls wdt_reset() every step, so a firmware with the watchdog on
// (hardware_probe_mega) does not reset during a long move. Stop the drive motors first:
// their own drive watchdog is not serviced while a blocking move runs.
//
// Example:
//   #include "../../core_robot_mega/include/stepper_driver.h"
//   static StepperDriver stepper;                    // default pins and 3200 steps per turn
//   void setup() { stepper.begin(); }
//   void loop()  { stepper.moveDegrees(360); delay(1000); stepper.moveDegrees(-360); delay(1000); }
#pragma once
#include <Arduino.h>
#include <avr/wdt.h>

class StepperDriver {
public:
  struct Config {
    uint8_t stepPin = 46, dirPin = 45, enPin = 44;   // PL3, PL4, PL5
    uint16_t stepsPerRev = 3200;                     // 200 x 1/16 microstepping
    uint16_t periodUs = 400;                         // cruise step period (400 us = 0.78 turn/s)
    uint16_t startPeriodUs = 1600;                   // first and last step period of a ramp
    uint16_t rampSteps = 400;                        // steps spent ramping up (and again down)
    bool holdTorque = false;                         // keep the driver on after a move
    bool invert = false;                             // flip which way is positive
  };

  StepperDriver() {}
  explicit StepperDriver(const Config& c) : cfg(c) {}

  // Call once from setup(). Leaves the driver disabled (motor free) until a move.
  void begin() {
    digitalWrite(cfg.enPin, HIGH); pinMode(cfg.enPin, OUTPUT);
    digitalWrite(cfg.stepPin, LOW); pinMode(cfg.stepPin, OUTPUT);
    digitalWrite(cfg.dirPin, LOW); pinMode(cfg.dirPin, OUTPUT);
    enabled = false;
  }

  // ---- settings ----
  void setSpeedUs(uint16_t periodUs) { cfg.periodUs = constrain(periodUs, 100, 50000); }
  void setRpm(float rpm) { if (rpm > 0) setSpeedUs(uint16_t(60e6f / (rpm * cfg.stepsPerRev))); }
  void setRamp(uint16_t startPeriodUs, uint16_t rampSteps) { cfg.startPeriodUs = startPeriodUs; cfg.rampSteps = rampSteps; }
  void setHold(bool on) { cfg.holdTorque = on; if (!busy()) enable(on); }
  void enable(bool on) { digitalWrite(cfg.enPin, on ? LOW : HIGH); if (on && !enabled) delay(2); enabled = on; }

  // ---- position (in steps since begin() or the last setZero()) ----
  long position() const { return pos; }
  float positionDegrees() const { return pos * 360.0f / cfg.stepsPerRev; }
  void setZero() { pos = 0; }
  long degreesToSteps(float deg) const { return lroundf(deg * cfg.stepsPerRev / 360.0f); }

  // ---- blocking moves ----
  // abort: optional function checked every step; return true to stop early (e.g. a STOP command).
  // Returns the number of steps actually made (signed).
  long moveSteps(long n, bool (*abort)() = nullptr) {
    startSteps(n);
    while (busy()) {
      wdt_reset();
      if (abort && abort()) { stop(); break; }
      service();
    }
    return done;
  }
  long moveDegrees(float deg, bool (*abort)() = nullptr) { return moveSteps(degreesToSteps(deg), abort); }
  long moveToDegrees(float deg, bool (*abort)() = nullptr) { return moveSteps(degreesToSteps(deg) - pos, abort); }

  // ---- non-blocking moves ----
  void startSteps(long n) {
    total = labs(n); left = total; done = 0;
    dirSign = n >= 0 ? 1 : -1;
    if (total == 0) { if (!cfg.holdTorque) enable(false); return; }
    digitalWrite(cfg.dirPin, ((n >= 0) != cfg.invert) ? HIGH : LOW);
    delayMicroseconds(5);                 // DIR setup time before the first STEP
    enable(true);
    lastStepUs = micros() - 60000UL;       // first step goes out on the next service()
  }
  void startDegrees(float deg) { startSteps(degreesToSteps(deg)); }

  // Call often while busy(). Returns true while the move is still running.
  bool service() {
    if (left == 0) return false;
    uint32_t now = micros();
    if (now - lastStepUs < nextPeriod()) return true;
    lastStepUs = now;
    digitalWrite(cfg.stepPin, HIGH);
    delayMicroseconds(3);                 // DRV8825 needs >= 1.9 us, A4988 >= 1 us
    digitalWrite(cfg.stepPin, LOW);
    --left; done += dirSign; pos += dirSign;
    if (left == 0 && !cfg.holdTorque) enable(false);
    return left != 0;
  }

  bool busy() const { return left != 0; }
  void stop() { left = 0; if (!cfg.holdTorque) enable(false); }   // immediate, no ramp down

private:
  // Linear ramp of the step period: startPeriodUs at both ends, periodUs in the middle.
  uint32_t nextPeriod() const {
    uint32_t i = total - left;            // index of the step about to be made
    uint32_t edge = i < left - 1 ? i : left - 1;   // steps from the nearer end of the move
    uint32_t ramp = cfg.rampSteps < total / 2 ? cfg.rampSteps : total / 2;
    if (cfg.startPeriodUs <= cfg.periodUs || ramp == 0 || edge >= ramp) return cfg.periodUs;
    return cfg.startPeriodUs - (uint32_t(cfg.startPeriodUs - cfg.periodUs) * edge) / ramp;
  }

  Config cfg;
  long pos = 0, done = 0;
  uint32_t total = 0, left = 0, lastStepUs = 0;
  int8_t dirSign = 1;
  bool enabled = false;
};

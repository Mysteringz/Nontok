#include "motion.h"
#include "motors.h"
#include "config.h"

static float cmdVx = 0, cmdVy = 0, cmdWz = 0;
static float holdTarget = 0;  // the heading is always held unless the Jetson asks to rotate
static bool turning = false;
static bool stopped = true;   // stopped by STOP or the watchdog, until the next VEL
static unsigned long lastVelMs = 0, turnStartMs = 0;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void motionBegin(float heading) {
  holdTarget = heading;
  motorsStop();
}

void motionSetVelocity(float vx, float vy, float wz, unsigned long now) {
  if (turning) return;  // the Jetson waits for DONE TURN before driving again
  cmdVx = vx;
  cmdVy = vy;
  cmdWz = wz;
  lastVelMs = now;
  stopped = false;
}

void motionHold(float headingDeg) { holdTarget = headingDeg; }

void motionTurn(float relDeg, unsigned long now) {
  cmdVx = cmdVy = cmdWz = 0;
  holdTarget += relDeg;  // after the turn the robot keeps holding the new heading
  turning = true;
  turnStartMs = now;
}

void motionStop() {
  cmdVx = cmdVy = cmdWz = 0;
  turning = false;
  stopped = true;
  motorsStop();
}

bool motionTurning() { return turning; }

MotionEvent motionUpdate(float heading, unsigned long now) {
  float err = holdTarget - heading;

  if (turning) {
    if (fabs(err) < TURN_TOLERANCE_DEG) {
      turning = false;
      stopped = true;
      motorsStop();
      return MOTION_TURN_DONE;
    }
    if (now - turnStartMs > TURN_TIMEOUT_MS) {
      turning = false;
      stopped = true;
      holdTarget = heading;
      motorsStop();
      return MOTION_TURN_TIMEOUT;
    }
    float rate = clampf(HEADING_KP * err, -TURN_MAX_RATE, TURN_MAX_RATE);
    if (fabs(rate) < TURN_MIN_RATE) rate = rate > 0 ? TURN_MIN_RATE : -TURN_MIN_RATE;
    motorsDrive(0, 0, rate);
    return MOTION_NONE;
  }

  if (stopped) return MOTION_NONE;
  if (cmdVx == 0 && cmdVy == 0 && cmdWz == 0) {
    motorsStop();  // parked: don't twitch to fix small heading errors
    return MOTION_NONE;
  }
  if (now - lastVelMs > CMD_WATCHDOG_MS) {  // Jetson stopped talking while moving
    motionStop();
    return MOTION_WATCHDOG;
  }

  float wz;
  if (cmdWz != 0) {
    wz = cmdWz;
    holdTarget = heading;  // rotating freely: hold whatever heading it ends at
  } else {
    wz = clampf(HEADING_KP * err, -HOLD_MAX_RATE, HOLD_MAX_RATE);
  }
  motorsDrive(cmdVx, cmdVy, wz);
  return MOTION_NONE;
}

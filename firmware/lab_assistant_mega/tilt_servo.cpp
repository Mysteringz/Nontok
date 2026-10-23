#include "tilt_servo.h"
#include "config.h"
#include <Servo.h>

static Servo servo;
static int current = TILT_DEFAULT_DEG;
static int target = TILT_DEFAULT_DEG;
static unsigned long lastStepMs = 0;

void tiltBegin() {
  servo.attach(TILT_SERVO_PIN);
  servo.write(current);
}

void tiltSet(int deg) { target = constrain(deg, TILT_MIN_DEG, TILT_MAX_DEG); }

void tiltUpdate(unsigned long now) {
  if (current == target || now - lastStepMs < TILT_STEP_MS) return;
  lastStepMs = now;
  current += (target > current) ? 1 : -1;
  servo.write(current);
}

int tiltCurrentDeg() { return current; }

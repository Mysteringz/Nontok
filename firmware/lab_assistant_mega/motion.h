// Motion control on top of the raw motors: velocity commands with a watchdog,
// gyro heading hold while driving, and closed-loop relative turns.
#pragma once
#include <Arduino.h>

enum MotionEvent { MOTION_NONE, MOTION_TURN_DONE, MOTION_TURN_TIMEOUT, MOTION_WATCHDOG };

void motionBegin(float heading);
void motionSetVelocity(float vx, float vy, float wz, unsigned long now);
void motionHold(float headingDeg);              // absolute gyro heading to hold
void motionTurn(float relDeg, unsigned long now);
void motionStop();
bool motionTurning();
MotionEvent motionUpdate(float heading, unsigned long now);  // call every loop

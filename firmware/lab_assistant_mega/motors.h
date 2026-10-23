#pragma once
#include <Arduino.h>

void motorsBegin();
// Body-frame velocity: vx forward (mm/s), vy left (mm/s), wz counter-clockwise (deg/s).
void motorsDrive(float vx, float vy, float wz);
void motorsStop();

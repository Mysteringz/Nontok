// Module 2: camera tilt servo, moved one degree at a time.
#pragma once
#include <Arduino.h>

void tiltBegin();
void tiltSet(int deg);
void tiltUpdate(unsigned long now);  // call every loop
int tiltCurrentDeg();

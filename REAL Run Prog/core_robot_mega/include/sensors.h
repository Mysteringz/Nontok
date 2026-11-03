#pragma once
#include "control.h"
#include <Arduino.h>
void sensorsBegin();
void sensorsUpdate();
core::Sensors sensorsRead();
const char* sensorsGyroState(); // GYRO_ZEROING_HOLD_STILL, GYRO_ZEROED, ...; announced on change.
void sensorsProbe(Stream& out); // Read-only I2C address scan, motors stopped first.

#pragma once
#include "control.h"
#include <Arduino.h>
void sensorsBegin();
void sensorsUpdate();
core::Sensors sensorsRead();
void sensorsProbe(Stream& out); // Read-only I2C address scan, motors stopped first.

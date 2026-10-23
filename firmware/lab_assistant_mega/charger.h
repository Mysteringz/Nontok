// Module 3: wireless charging receiver. The Jetson treats a high reading as "on the coil".
#pragma once
#include <Arduino.h>

void chargerBegin();
uint16_t chargerMv();  // receiver output voltage in mV

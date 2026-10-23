// Module 1: HC-SR04 ultrasonic sensors (front, left, right).
#pragma once
#include <Arduino.h>

void ultrasonicBegin();
void ultrasonicUpdate();            // pings one sensor per call (round robin)
uint16_t ultrasonicMm(uint8_t idx); // latest distance, US_MAX_MM if no echo

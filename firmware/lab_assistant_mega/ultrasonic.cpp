#include "ultrasonic.h"
#include "config.h"

static uint16_t distances[US_COUNT];
static uint8_t nextSensor = 0;

void ultrasonicBegin() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    pinMode(US_ECHO[i], INPUT);
    digitalWrite(US_TRIG[i], LOW);
    distances[i] = US_MAX_MM;
  }
}

// One sensor per call: the sensors never hear each other's echoes, and the loop
// blocks for at most one timeout.
void ultrasonicUpdate() {
  uint8_t i = nextSensor;
  nextSensor = (nextSensor + 1) % US_COUNT;

  digitalWrite(US_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG[i], LOW);

  unsigned long us = pulseIn(US_ECHO[i], HIGH, US_TIMEOUT_US);
  // Sound travels 0.343 mm/us; halve for the round trip.
  // TODO: add a median-of-3 filter if readings spike.
  distances[i] = (us == 0) ? US_MAX_MM : (uint16_t)min(us * 0.1715, (double)US_MAX_MM);
}

uint16_t ultrasonicMm(uint8_t idx) { return distances[idx]; }

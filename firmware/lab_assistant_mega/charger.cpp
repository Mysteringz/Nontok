#include "charger.h"
#include "config.h"

void chargerBegin() { pinMode(CHARGE_SENSE_PIN, INPUT); }

uint16_t chargerMv() {
  return (uint16_t)(analogRead(CHARGE_SENSE_PIN) * (5000.0 / 1023.0) * CHARGE_DIVIDER_RATIO);
}

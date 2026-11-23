// Stepper bench test, using the shared stepper_driver.h (wiring and steps per turn are documented there).
// Drive motors are held off. At boot: +90 deg, 1 s pause, -90 deg.
// Serial 115200 commands:  "S n" move n steps   "D deg" move degrees   "V us" cruise step period
//                          "E 1" / "E 0" hold torque on / off   "P" print position   "Z" zero position
//                          "!" stops a move
#include <Arduino.h>
#include <avr/wdt.h>
#include "../../core_robot_mega/include/stepper_driver.h"

static const uint8_t motorPwm[]={12,8,9,5}, motorDir[]={34,35,37,36,43,42,A4,A5};
static StepperDriver stepper;

static bool bangPressed() {
  if (Serial.available() && Serial.peek() == '!') { Serial.read(); Serial.println(F("stopped")); return true; }
  return false;
}
static void report(long made) {
  Serial.print(F("moved ")); Serial.print(made);
  Serial.print(F("  pos ")); Serial.print(stepper.position());
  Serial.print(F(" steps = ")); Serial.print(stepper.positionDegrees(), 1); Serial.println(F(" deg"));
}

void setup() {
  wdt_disable();
  for (uint8_t p : motorPwm) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  for (uint8_t p : motorDir) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  stepper.begin();
  Serial.begin(115200);
  Serial.println(F("STEPPER TEST (stepper_driver.h)  STEP=PL3/D46 DIR=PL4/D45 EN=PL5/D44, 3200 steps/turn"));
  Serial.println(F("cmds: S n | D deg | V us | E 1/0 | P | Z | ! stops a move"));
  delay(1500);
  report(stepper.moveDegrees(90, bangPressed));
  delay(1000);
  report(stepper.moveDegrees(-90, bangPressed));
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line.length() < 1) return;
  char c = toupper(line[0]); String arg = line.substring(1); arg.trim();
  if (c == 'S') report(stepper.moveSteps(arg.toInt(), bangPressed));
  else if (c == 'D') report(stepper.moveDegrees(arg.toFloat(), bangPressed));
  else if (c == 'V') { stepper.setSpeedUs(arg.toInt()); Serial.println(F("ok")); }
  else if (c == 'E') { stepper.setHold(arg.toInt() != 0); Serial.println(F("ok")); }
  else if (c == 'P') report(0);
  else if (c == 'Z') { stepper.setZero(); report(0); }
  else Serial.println(F("? S n | D deg | V us | E 1/0 | P | Z"));
}

// Arduino Mega firmware for the self-driving lab assistant.
// The Mega drives the motors and reads the sensors; all decisions are made on the Jetson.
// Line protocol over USB serial: see docs/lab_assistant_protocol.md.
// (docs/serial_protocol.md is the core sketch's CORE1 protocol, not this one.)

#include "config.h"
#include "motors.h"
#include "motion.h"
#include "ultrasonic.h"
#include "imu.h"
#include "tilt_servo.h"
#include "charger.h"

static char lineBuf[64];
static uint8_t lineLen = 0;
static unsigned long lastTelemetryMs = 0, lastPingMs = 0;

static float nextFloat() {
  char* tok = strtok(NULL, " ");
  return tok ? atof(tok) : 0.0;
}

static void handleCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  unsigned long now = millis();

  if (!strcmp(cmd, "VEL")) {  // no reply: sent ~10 times a second
    float vx = nextFloat(), vy = nextFloat(), wz = nextFloat();
    motionSetVelocity(vx, vy, wz, now);
  } else if (!strcmp(cmd, "TURN")) {
    motionTurn(nextFloat(), now);
    Serial.println(F("OK TURN"));
  } else if (!strcmp(cmd, "HOLD")) {
    motionHold(nextFloat());
    Serial.println(F("OK HOLD"));
  } else if (!strcmp(cmd, "STOP")) {
    motionStop();
    Serial.println(F("OK STOP"));
  } else if (!strcmp(cmd, "TILT")) {
    tiltSet((int)nextFloat());
    Serial.println(F("OK TILT"));
  } else if (!strcmp(cmd, "ZERO")) {
    imuZero();
    motionHold(0);
    Serial.println(F("OK ZERO"));
  } else if (!strcmp(cmd, "CAL")) {
    motionStop();
    imuCalibrate();
    Serial.println(F("OK CAL"));
  } else if (!strcmp(cmd, "PING")) {
    Serial.println(F("PONG"));
  } else {
    Serial.print(F("ERR UNKNOWN "));
    Serial.println(cmd);
  }
}

static void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overlong line: drop it
      Serial.println(F("ERR OVERFLOW"));
    }
  }
}

static void sendTelemetry(unsigned long now) {
  // TEL <t_ms> <heading_deg> <front_mm> <left_mm> <right_mm> <charge_mv> <turning> <tilt_deg>
  Serial.print(F("TEL "));
  Serial.print(now);
  Serial.print(' ');
  Serial.print(imuHeadingDeg(), 1);
  Serial.print(' ');
  Serial.print(ultrasonicMm(US_FRONT));
  Serial.print(' ');
  Serial.print(ultrasonicMm(US_LEFT));
  Serial.print(' ');
  Serial.print(ultrasonicMm(US_RIGHT));
  Serial.print(' ');
  Serial.print(chargerMv());
  Serial.print(' ');
  Serial.print(motionTurning() ? 1 : 0);
  Serial.print(' ');
  Serial.println(tiltCurrentDeg());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  motorsBegin();
  ultrasonicBegin();
  tiltBegin();
  chargerBegin();
  if (!imuBegin()) Serial.println(F("ERR IMU_NOT_FOUND"));
  motionBegin(imuHeadingDeg());
  Serial.println(F("READY"));
}

void loop() {
  unsigned long now = millis();
  readSerial();
  imuUpdate();

  if (now - lastPingMs >= US_PERIOD_MS) {
    lastPingMs = now;
    ultrasonicUpdate();
  }

  switch (motionUpdate(imuHeadingDeg(), now)) {
    case MOTION_TURN_DONE: Serial.println(F("DONE TURN")); break;
    case MOTION_TURN_TIMEOUT: Serial.println(F("ERR TURN_TIMEOUT")); break;
    case MOTION_WATCHDOG: Serial.println(F("ERR WATCHDOG")); break;
    default: break;
  }

  tiltUpdate(now);

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry(now);
  }
}

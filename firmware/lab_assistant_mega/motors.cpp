#include "motors.h"
#include "config.h"

static void setMotor(uint8_t i, float wheelMmS) {
  float s = wheelMmS * MOTOR_SIGN[i];
  float frac = fabs(s) / MAX_WHEEL_SPEED_MM_S;
  if (frac > 1.0) frac = 1.0;
  int pwm = (fabs(s) < 1.0) ? 0 : MIN_PWM + (int)((MAX_PWM - MIN_PWM) * frac);
  digitalWrite(MOTOR_DIR_A[i], s >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_DIR_B[i], s >= 0 ? LOW : HIGH);
  analogWrite(MOTOR_PWM[i], pwm);
}

void motorsBegin() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_PWM[i], OUTPUT);
    pinMode(MOTOR_DIR_A[i], OUTPUT);
    pinMode(MOTOR_DIR_B[i], OUTPUT);
  }
  motorsStop();
}

void motorsDrive(float vx, float vy, float wz) {
  // Mecanum inverse kinematics (rollers in an X when viewed from above).
  float w = wz * DEG_TO_RAD * ROTATION_ARM_MM;
  float speeds[4] = {
    vx - vy - w,  // front-left
    vx + vy + w,  // front-right
    vx + vy - w,  // rear-left
    vx - vy + w,  // rear-right
  };
  // Scale all wheels together so the direction of travel is kept when one saturates.
  float peak = 0;
  for (uint8_t i = 0; i < 4; i++) peak = max(peak, fabs(speeds[i]));
  float scale = peak > MAX_WHEEL_SPEED_MM_S ? MAX_WHEEL_SPEED_MM_S / peak : 1.0;
  for (uint8_t i = 0; i < 4; i++) setMotor(i, speeds[i] * scale);
}

void motorsStop() {
  for (uint8_t i = 0; i < 4; i++) analogWrite(MOTOR_PWM[i], 0);
}

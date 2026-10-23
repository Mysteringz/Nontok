#include "imu.h"
#include "config.h"
#include <Wire.h>

static const float LSB_PER_DPS = 131.0;  // +-250 deg/s full scale
static float biasZ = 0;
static float heading = 0;
static unsigned long lastUs = 0;

static void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static int16_t readGyroZRaw() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)2);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

bool imuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1: wake up
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;
  writeReg(0x1B, 0x00);  // GYRO_CONFIG: +-250 deg/s
  writeReg(0x1A, 0x03);  // CONFIG: ~44 Hz low-pass filter
  delay(100);
  imuCalibrate();
  return true;
}

void imuCalibrate() {
  long sum = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  biasZ = (float)sum / GYRO_CAL_SAMPLES;
  lastUs = micros();
}

void imuUpdate() {
  unsigned long now = micros();
  float dt = (now - lastUs) * 1e-6;
  lastUs = now;
  float rate = IMU_SIGN * (readGyroZRaw() - biasZ) / LSB_PER_DPS;
  if (fabs(rate) < GYRO_DEADBAND_DPS) rate = 0;  // limits drift while parked
  heading += rate * dt;
}

void imuZero() { heading = 0; }

float imuHeadingDeg() { return heading; }

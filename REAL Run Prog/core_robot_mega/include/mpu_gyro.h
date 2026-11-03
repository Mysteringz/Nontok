#pragma once
// InvenSense MPU-60x0/65xx-compatible register map shared by the core and probe.
// Pure helpers only, so host tests can exercise them without Arduino headers.
#include <stdint.h>
namespace core {
const uint8_t MPU_SMPLRT_DIV=0x19,MPU_CONFIG=0x1a,MPU_GYRO_CONFIG=0x1b,MPU_FIFO_EN=0x23;
const uint8_t MPU_INT_PIN_CFG=0x37,MPU_INT_ENABLE=0x38,MPU_INT_STATUS=0x3a;
const uint8_t MPU_ACCEL_XOUT=0x3b,MPU_GYRO_XOUT=0x43,MPU_USER_CTRL=0x6a,MPU_PWR_MGMT_1=0x6b;
const uint8_t MPU_FIFO_COUNT=0x72,MPU_FIFO_RW=0x74,MPU_WHO_AM_I=0x75;
const uint8_t MPU_FIFO_GYRO_BIT[3]={0x40,0x20,0x10}; // XG, YG, ZG in FIFO_EN.
const uint8_t MPU_INT_FIFO_OVERFLOW=0x10,MPU_INT_DATA_READY=0x01;
const uint8_t MPU_SAMPLE_DIVIDER=4;           // 1 kHz DLPF clock / (1+4).
const float MPU_SAMPLE_HZ=200.0f;            // Every sample is buffered in the FIFO and integrated.
const uint16_t MPU_ZERO_SAMPLES=400;          // 2 s bias window.
const float MPU_GYRO_LSB_PER_DPS=65.5f;       // GYRO_CONFIG FS_SEL=1: +/-500 deg/s.
const int16_t MPU_SATURATION=32700;
const uint16_t MPU_FIFO_BYTES_MIN=512;        // MPU6500 family; MPU6050 has 1024.

struct MpuWrite { uint8_t reg,value; };
// Wake on the gyro PLL, 200 Hz samples, ~44 Hz DLPF, +/-500 deg/s.
// INT is active-high push-pull, latched until any register read.
const MpuWrite MPU_SETUP[]={
  {MPU_PWR_MGMT_1,0x01},{MPU_SMPLRT_DIV,MPU_SAMPLE_DIVIDER},{MPU_CONFIG,0x03},
  {MPU_GYRO_CONFIG,0x08},{MPU_INT_PIN_CFG,0x30},
  {MPU_INT_ENABLE,MPU_INT_FIFO_OVERFLOW|MPU_INT_DATA_READY},
  {MPU_FIFO_EN,0},{MPU_USER_CTRL,0x04}};       // FIFO off and reset; enable later.
const uint8_t MPU_USER_FIFO_ON=0x40,MPU_USER_FIFO_RESET=0x04;

// WHO_AM_I values of parts sharing the registers above. Unknown IDs are not configured.
inline const char* mpuModel(uint8_t who) {
  switch(who) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x72: return "MPU6050_COMPATIBLE_0x72";
    case 0x73: return "MPU9255";
    case 0x12: return "ICM20602";
    case 0x98: return "ICM20689";
    case 0xaf: return "ICM20608";
    default: return 0;
  }
}
inline int16_t mpuWord(const uint8_t* bytes) { return int16_t((uint16_t(bytes[0])<<8)|bytes[1]); }
inline bool mpuSaturated(int16_t raw) { return raw>=MPU_SATURATION||raw<=-MPU_SATURATION; }
// Degrees contributed by one FIFO sample after bias removal.
inline float mpuSampleDegrees(int16_t raw,float bias) {
  return (raw-bias)/(MPU_GYRO_LSB_PER_DPS*MPU_SAMPLE_HZ);
}
}

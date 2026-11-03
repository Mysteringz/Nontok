#pragma once
#include <Arduino.h>

// Confirm the actual wheel wiring and sensor connections before enabling them.
#ifndef MOTOR_WIRING_CONFIRMED
#define MOTOR_WIRING_CONFIRMED 0
#endif
#ifndef SENSOR_WIRING_CONFIRMED
#define SENSOR_WIRING_CONFIRMED 0
#endif
#ifndef MOEBIUS_LEGACY_M4_PINS
#define MOEBIUS_LEGACY_M4_PINS 0
#endif
#if MOEBIUS_LEGACY_M4_PINS != 0 && MOEBIUS_LEGACY_M4_PINS != 1
#error "MOEBIUS_LEGACY_M4_PINS must be 0 or 1"
#endif

// Order: front-left/M1, front-right/M2, rear-left/M3, rear-right/M4.
const uint8_t MOTOR_PWM[4] = {12, 8, 9, 5};
#if MOEBIUS_LEGACY_M4_PINS
const uint8_t MOTOR_IN1[4] = {34, 37, 43, 26};
const uint8_t MOTOR_IN2[4] = {35, 36, 42, 27};
#else
const uint8_t MOTOR_IN1[4] = {34, 37, 43, A4};
const uint8_t MOTOR_IN2[4] = {35, 36, 42, A5};
#endif
// Measured 23 Oct with wheels raised (dashboard identify): FL=M1, FR=M2, RL=M3, RR=M4;
// wheel-forward (toward the ultrasonic end) is raw + on M1/M3 and raw - on M2/M4.
// This is the negation of the old sketch's ADVANCE, which drove toward the other end.
const int8_t MOTOR_SIGN[4] = {1, -1, 1, -1};
const float MAX_WHEEL_MM_S = 400; // Nominal PWM calibration, not odometry.
const float ROTATION_ARM_MM = 160; // Half track + half wheelbase; measure.
const uint8_t MIN_PWM = 60, MAX_PWM = 120;

// Working sketch: Bluetooth Serial3, TX3 D14 / RX3 D15, 9600 baud.
// 0: shared Serial0/USB. 2: Serial2. 3: Serial3.
// If using 0, USB and Bluetooth must not drive the RX line simultaneously.
#ifndef BLUETOOTH_UART
#define BLUETOOTH_UART 3
#endif
#if BLUETOOTH_UART != 0 && BLUETOOTH_UART != 2 && BLUETOOTH_UART != 3
#error "BLUETOOTH_UART must be 0, 2 or 3"
#endif
const unsigned long USB_BAUD = 115200;
// Serial0 has one baud: a COM0 Bluetooth module must also be set to 115200.
const unsigned long BLUETOOTH_BAUD = BLUETOOTH_UART == 0 ? USB_BAUD : 9600;

// User-reported 2026-10-23 (plugs swapped after a left-cable fault):
// left PL1/PL2 = D48 TRIG / D47 ECHO, right PG1/PG0 = D40 TRIG / D41 ECHO.
#ifndef RANGE_LEFT_ECHO_PIN
#define RANGE_LEFT_ECHO_PIN 47
#endif
const uint8_t RANGE_TRIG[2] = {48, 40};
const uint8_t RANGE_ECHO[2] = {RANGE_LEFT_ECHO_PIN, 41};
// Front ultrasonic, added 27 Oct: TRIG PA0 = D22, ECHO PA2 = D24. Read by the hardware probe;
// the mission, localization and MEGA3 telemetry still use only left/right.
const uint8_t RANGE_FRONT_TRIG = 22, RANGE_FRONT_ECHO = 24;
// Side ultrasonics, wired 27 Oct, {left facing +90 deg, right facing -90 deg}: TRIG PB4/PH3,
// ECHO PB5/PH4 on the SERVO row. Probe only, like the front sensor.
const uint8_t RANGE_SIDE_TRIG[2] = {10, 6}, RANGE_SIDE_ECHO[2] = {11, 7};
// Four LDR channels, physically left to right on A0/A2/A7/A6 (confirmed 27 Oct; LDR_PIN below)
// (A1 and A3 not used; A4/A5 belong to M4). Four channels, NOT three identified beacons.
constexpr uint8_t LIGHT_SENSOR_PIN[4] = {A0,A7,A2,A6};
static_assert(LIGHT_SENSOR_PIN[0]!=A4&&LIGHT_SENSOR_PIN[1]!=A4&&LIGHT_SENSOR_PIN[2]!=A4&&LIGHT_SENSOR_PIN[3]!=A4&&
  LIGHT_SENSOR_PIN[0]!=A5&&LIGHT_SENSOR_PIN[1]!=A5&&LIGHT_SENSOR_PIN[2]!=A5&&LIGHT_SENSOR_PIN[3]!=A5,
  "Light inputs cannot use A4/A5: they are the M4 direction pins on the updated shield");
// Each channel must measure ONE distinguishable beacon. Combined brightness is unsupported.
#ifndef LIGHT_DRIVER
#define LIGHT_DRIVER 0
#endif
#if LIGHT_DRIVER < 0 || LIGHT_DRIVER > 1
#error "LIGHT_DRIVER: 0 unidentified, 1 three independent analog strength channels"
#endif
// Legacy adapter proposal only; the reported four-channel interface is not decoded yet.
const uint8_t LIGHT_PIN[3] = {A6,A7,A8}; // left, middle, right resolved beacon strengths
const bool LIGHT_HIGH_IS_BRIGHT = true; // Measured 27 Oct (flashlight raises the reading); reverses 23 Oct.
const uint8_t LDR_PIN[4] = {A0,A2,A7,A6}; // The four installed LDRs, left to right (confirmed 27 Oct); not yet decoded.
const uint8_t IMU_ADDRESS = 0x68; // MPU6050 identified 2026-10-23 on the shield IMU header (D20/D21), INT D33.
const float GYRO_SIGN = -1.0f; // Positive CCW. Confirmed 27 Oct by hand: 4 x 90 deg left read raw Z -357.4 deg.
// 0 off; 1 TCS34725 I2C; 2 TCS230/TCS3200-compatible frequency module.
// IMG_3849 shows GND/OUT/S2/S3/VCC: select 2, but wiring stays gated below.
#ifndef COLOR_DRIVER
#define COLOR_DRIVER 2
#endif
#if COLOR_DRIVER < 0 || COLOR_DRIVER > 2
#error "COLOR_DRIVER must be 0, 1 or 2"
#endif
const uint8_t COLOR_ADDRESS = 0x29;
// User reports all five signal endpoints and confirms a four-wire OLED (no reset).
// D28 is available for S0. S0=HIGH/S1=LOW selects 20% frequency scaling.
#ifndef COLOR_SCALE_PINS_EXPOSED
#define COLOR_SCALE_PINS_EXPOSED 1
#endif
#if COLOR_SCALE_PINS_EXPOSED != 0 && COLOR_SCALE_PINS_EXPOSED != 1
#error "COLOR_SCALE_PINS_EXPOSED must be 0 or 1"
#endif
const uint8_t COLOR_S2=30,COLOR_S3=29,COLOR_OUT=32; // PC7, PA7, PC5.
#if SENSOR_WIRING_CONFIRMED && COLOR_DRIVER == 2
static_assert(RANGE_LEFT_ECHO_PIN!=COLOR_OUT,
  "Colour OUT cannot share the left ultrasonic ECHO input");
#endif
const uint8_t COLOR_S0=28,COLOR_S1=25; // PA6, PA3, as reported by user.
#ifndef OLED_RESET_ON_D28
#define OLED_RESET_ON_D28 0
#endif
#if SENSOR_WIRING_CONFIRMED && COLOR_DRIVER == 2 && COLOR_SCALE_PINS_EXPOSED && OLED_RESET_ON_D28
#error "Colour S0 D28 cannot also drive a physically connected OLED reset wire"
#endif
// Driver 2 reserves D32/PC5; no timer is reconfigured by colour sampling.
// Serial3 Bluetooth and I2C OLED stay unchanged. D19 is no longer reserved.
// All sensor hardware access remains disabled until SENSOR_WIRING_CONFIRMED=1.

// 0 off; 1 SSD1306 128x64; 2 SH1106 128x64; 3 SSD1306 128x32 (working sketch).
#ifndef OLED_DRIVER
#define OLED_DRIVER 0
#endif
#if OLED_DRIVER < 0 || OLED_DRIVER > 3
#error "OLED_DRIVER must be 0, 1, 2 or 3"
#endif
const uint8_t OLED_ADDRESS = 0x3c; // Mega SDA=20, SCL=21.

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
const int8_t MOTOR_SIGN[4] = {-1, -1, -1, -1}; // Verify each wheel raised off floor.
const float MAX_WHEEL_MM_S = 400; // Nominal PWM calibration, not odometry.
const float ROTATION_ARM_MM = 160; // Half track + half wheelbase; measure.
const uint8_t MIN_PWM = 60, MAX_PWM = 120;

// 0: Bluetooth shares Serial0/USB (vendor default). 2: Serial2. 3: Serial3.
// If using 0, USB and Bluetooth must not drive the RX line simultaneously.
#ifndef BLUETOOTH_UART
#define BLUETOOTH_UART 0
#endif
#if BLUETOOTH_UART != 0 && BLUETOOTH_UART != 2 && BLUETOOTH_UART != 3
#error "BLUETOOTH_UART must be 0, 2 or 3"
#endif
const unsigned long USB_BAUD = 115200;
// Serial0 has one baud: a COM0 Bluetooth module must also be set to 115200.
const unsigned long BLUETOOTH_BAUD = BLUETOOTH_UART == 0 ? USB_BAUD : 9600;

// Proposed free pins: verify against the installed cables before enabling.
const uint8_t RANGE_TRIG[2] = {22, 24};
const uint8_t RANGE_ECHO[2] = {30, 25}; // Avoid legacy encoder pin 23.
// Each channel must measure ONE distinguishable beacon. Combined brightness is unsupported.
#ifndef LIGHT_DRIVER
#define LIGHT_DRIVER 0
#endif
#if LIGHT_DRIVER < 0 || LIGHT_DRIVER > 1
#error "LIGHT_DRIVER: 0 unidentified, 1 three independent analog strength channels"
#endif
const uint8_t LIGHT_PIN[3] = {A6,A7,A8}; // left, middle, right beacon
const bool LIGHT_HIGH_IS_BRIGHT = true;
const uint8_t IMU_ADDRESS = 0x68; // MPU6050, verify/probe before confirmation.
const float GYRO_SIGN = 1.0f; // Positive CCW, check physical mounting.
// 0 off; 1 TCS34725 I2C; 2 TCS230/TCS3200-compatible frequency module.
// IMG_3849 shows GND/OUT/S2/S3/VCC: select 2, but wiring stays gated below.
#ifndef COLOR_DRIVER
#define COLOR_DRIVER 2
#endif
#if COLOR_DRIVER < 0 || COLOR_DRIVER > 2
#error "COLOR_DRIVER must be 0, 1 or 2"
#endif
const uint8_t COLOR_ADDRESS = 0x29;
// Use the visible five-pin header without driving unverified S0/S1/OE connections.
// Their location, enabled levels and scaling must be verified on the full board.
#ifndef COLOR_SCALE_PINS_EXPOSED
#define COLOR_SCALE_PINS_EXPOSED 0
#endif
#if COLOR_SCALE_PINS_EXPOSED != 0 && COLOR_SCALE_PINS_EXPOSED != 1
#error "COLOR_SCALE_PINS_EXPOSED must be 0 or 1"
#endif
const uint8_t COLOR_S2=44,COLOR_S3=45,COLOR_OUT=47; // D47/PL2 = Timer5 external clock T5.
#if COLOR_SCALE_PINS_EXPOSED
const uint8_t COLOR_S0=40,COLOR_S1=41; // Optional full-header module only, 20% scaling.
#endif
// Driver 2 exclusively owns Timer5. No Servo library / PWM 44,45,46 in this sketch.
// All sensor hardware access remains disabled until SENSOR_WIRING_CONFIRMED=1.

// 0 off; 1 SSD1306 128x64 I2C; 2 SH1106 128x64 I2C. Verify display first.
#ifndef OLED_DRIVER
#define OLED_DRIVER 0
#endif
#if OLED_DRIVER < 0 || OLED_DRIVER > 2
#error "OLED_DRIVER must be 0, 1 or 2"
#endif
const uint8_t OLED_ADDRESS = 0x3c; // Mega SDA=20, SCL=21.

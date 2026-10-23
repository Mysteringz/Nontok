// Pin map and tuning constants for the Arduino Mega.
// Units: mm, mm/s, degrees, deg/s, ms. Anything marked TODO must be checked on the real robot.
#pragma once
#include <Arduino.h>

// ---------------- Serial link to the Jetson (docs/lab_assistant_protocol.md) ----------------
#define SERIAL_BAUD 115200
#define TELEMETRY_PERIOD_MS 50  // 20 Hz
#define CMD_WATCHDOG_MS 500     // motors stop if no VEL arrives for this long

// ---------------- Mecanum drive ----------------
// Wheel order everywhere: front-left, front-right, rear-left, rear-right.
// Pins and signs are copied from firmware/core_robot_mega/config.h (the candidate
// "4 Motor & 9 Servo" TB6612 profile in docs/hardware.md). That sketch is where the
// wiring gets confirmed: copy any correction from there into here.
const uint8_t MOTOR_PWM[4]   = {12, 8, 9, 5};
const uint8_t MOTOR_DIR_A[4] = {35, 36, 42, A5};  // IN1
const uint8_t MOTOR_DIR_B[4] = {34, 37, 43, A4};  // IN2
const int8_t  MOTOR_SIGN[4]  = {1, -1, 1, -1};

#define MAX_WHEEL_SPEED_MM_S 400.0  // wheel speed at MAX_PWM, TODO: measure
#define MIN_PWM 60                  // below this the motors stall
#define MAX_PWM 120                 // same commissioning cap as the core sketch; raise after floor tests
#define ROTATION_ARM_MM 160.0       // half wheelbase + half track width, TODO: measure

// ---------------- Heading control (gyro) ----------------
#define HEADING_KP 3.0           // deg/s of correction per degree of error
#define HOLD_MAX_RATE 30.0       // max correction while driving straight
#define TURN_MAX_RATE 90.0
#define TURN_MIN_RATE 20.0       // below this the robot does not rotate at all
#define TURN_TOLERANCE_DEG 2.0
#define TURN_TIMEOUT_MS 6000

// ---------------- Ultrasonic HC-SR04 (module 1) ----------------
enum { US_FRONT = 0, US_LEFT, US_RIGHT, US_COUNT };
// TODO: wiring. Keep clear of the motor pins above (34-37, 42, 43 are motor direction pins).
const uint8_t US_TRIG[US_COUNT] = {22, 24, 26};
const uint8_t US_ECHO[US_COUNT] = {23, 25, 27};
#define US_PERIOD_MS 20          // one sensor per period, round robin
#define US_TIMEOUT_US 25000      // ~4.3 m
#define US_MAX_MM 4000           // reported when nothing echoes back

// ---------------- MPU6050 gyroscope (module 1) ----------------
#define IMU_ADDR 0x68
#define IMU_SIGN 1.0             // -1 if the board is mounted upside down
#define GYRO_CAL_SAMPLES 500     // robot must be still during calibration
#define GYRO_DEADBAND_DPS 0.3

// ---------------- Camera tilt servo (module 2) ----------------
#define TILT_SERVO_PIN 10        // TODO: match a servo header on the shield (9 is motor C PWM).
                                 // Servo.h uses Timer5: no PWM on 44-46 while it runs
#define TILT_MIN_DEG 0
#define TILT_MAX_DEG 180
#define TILT_DEFAULT_DEG 90
#define TILT_STEP_MS 15          // 1 degree per step, so the camera does not jerk

// ---------------- Wireless charging receiver sense (module 3) ----------------
#define CHARGE_SENSE_PIN A0
#define CHARGE_DIVIDER_RATIO 2.0  // receiver output -> resistor divider -> A0, TODO: match divider

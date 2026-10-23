// Module 1: MPU6050 gyroscope, integrated into a heading (deg, counter-clockwise positive).
#pragma once
#include <Arduino.h>

bool imuBegin();       // false if the MPU6050 does not answer on I2C
void imuCalibrate();   // measures gyro bias, the robot must be still
void imuUpdate();      // call every loop
void imuZero();
float imuHeadingDeg(); // not wrapped: keeps counting past +-180

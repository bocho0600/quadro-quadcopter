#ifndef MPUINIT_H
#define MPUINIT_H
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>

#define MPU9250_ADDRESS 0x68 // Address of MPU9250 0b01101000 - 104
#define WHO_AM_I 0x75        // Register to read the device ID 0b01110101
#define PWR_MGMT_1 0x6B      // Register to write to power management 0b01101011 - 107
#define ACCEL_XOUT_H 0x3B    // Register to read X-acceleration data 0b00111011
#define GYRO_XOUT_H 0x43     // Register to read X-gyroscope data 0b01000011 (register 67)
#define MAG_ADDRESS 0x0C     // Address of magnetometer 0b00001100

/* The rotation of the MPU9250 is defined as follows, with:
Roll - rotation around X-axis
Pitch - rotation around Y-axis
Yaw - rotation around Z-axis (counter-clockwise is positive)
*/
extern float RateRoll, RatePitch, RateYaw; // Gyroscope data

//Gyroscope functions
void GyroInit();
void MPU9250_LOWPASS(); // Activate the low-pass filter
void GyroResponse();    // Read the gyroscope data
void GyroConfiguration();
void PowerManagementMPU(); // Wake up the MPU9250

#endif
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
#include "AccelInit.h"
#define MPU9250_ADDRESS 0x68 // Address of MPU9250 0b01101000 - 104
#define WHO_AM_I 0x75        // Register to read the device ID 0b01110101
#define PWR_MGMT_1 0x6B      // Register to write to power management 0b01101011 - 107
#define ACCEL_XOUT_H 0x3B    // Register to read X-acceleration data 0b00111011
#define GYRO_XOUT_H 0x43     // Register to read X-gyroscope data 0b01000011 (register 67)
#define MAG_ADDRESS 0x0C     // Address of magnetometer 0b00001100

// Define global variables
float AccX = 0.0, AccY = 0.0, AccZ = 0.0;
float AngleRoll = 0.0, AnglePitch = 0.0;
void AccelConfiguration()
{
      Wire.beginTransmission(MPU9250_ADDRESS);
      Wire.write(0x1C);       // Access register 1C (28) - Accelerometer Configuration
      Wire.write(0b00010000); // Set the sensitivity of the accelerometer to 8g (8192 LSB/g)
      Wire.endTransmission();
}
void AccelResponse()
{
      Wire.beginTransmission(MPU9250_ADDRESS);
      Wire.write(ACCEL_XOUT_H); // Access register 59 - Accelerometer X-axis data
      Wire.endTransmission();
      Wire.requestFrom(MPU9250_ADDRESS, 6); // Request 6 bytes of data from register 59 to 64
      if (Wire.available() == 6)
      {
            int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // Read the X-axis accelerometer data
            int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // Read the Y-axis accelerometer data
            int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // Read the Z-axis accelerometer data
            AccX = (float)AccXLSB / 4096 + 0.02;              // Convert the data to g from 4096 LSB/g
            AccY = (float)AccYLSB / 4096;
            AccZ = (float)AccZLSB / 4096 - 0.02;                                          // calibraion
            AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180); // Convert the data to degrees
            AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
      }
}
void AccelRead()
{
      AccelConfiguration();
      AccelResponse();
}
void AccelPrint(uint8_t type)
{
      if (type == 1)
      {
            Serial.print("AccX: ");
            Serial.print(AccX);
            Serial.print(" AccY: ");
            Serial.print(AccY);
            Serial.print(" AccZ: ");
            Serial.print(AccZ);
      }
      else if (type == 2)
      {
            Serial.print(" AngleRoll: ");
            Serial.print(AngleRoll);
            Serial.print(" AnglePitch: ");
            Serial.print(AnglePitch);

      }
}

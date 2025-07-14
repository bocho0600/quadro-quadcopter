#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "AccelInit.h"
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
State state = CALIBRATION;
int RateCalibrationNumber = 0;                                       // Number of calibration data
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // Calibration data
float RateRoll, RatePitch, RateYaw, temp;                                  // Gyroscope data

void MPU9250_LOWPASS() // Activate Low-pass filter to limit the rate of noise
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1A); // Access register 1A (26) contain the DLPF setting
  Wire.write(0x05); // Set the DLPF to 10Hz bandwidth
  Wire.endTransmission();
}

void GyroInit()
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B);       // Access register 1B (27) - Gyroscope Configuration
  Wire.write(0b00001000); // Set the sensitivity of the gyroscope to 500 degrees per second (65 LSB/s)
  Wire.endTransmission();
}

void GyroResponse()
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(GYRO_XOUT_H); // Access register 43 - Gyroscope X-axis data
  Wire.endTransmission();
  Wire.requestFrom(MPU9250_ADDRESS, 6); // Request 6 bytes of data from register 43 to 48
  if (Wire.available() == 6)
  {
    int16_t GyroX = Wire.read() << 8 | Wire.read(); // Read the X-axis gyroscope data
    int16_t GyroY = Wire.read() << 8 | Wire.read(); // Read the Y-axis gyroscope data
    int16_t GyroZ = Wire.read() << 8 | Wire.read(); // Read the Z-axis gyroscope data
    RateRoll = (float)GyroX / 65.5;                        // Convert the data to degrees per second from 65.5 LSB/s
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    // temp = RateRoll;
    // RatePitch = temp;
    // RateRoll = RatePitch; // Set RateRoll to RatePitch for testing purposes
  }
}

void GyroConfiguration()
{
  MPU9250_LOWPASS(); // Activate the low-pass filter
  GyroInit();        // Initialize the gyroscope
  GyroResponse();    // Read the gyroscope data
}

void PowerManagementMPU()
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(PWR_MGMT_1); // Access register 6B (107) - Power Management 1
  Wire.write(0x00);       // Set the power management to 0 to wake up the MPU9250
  Wire.endTransmission();
}

void MPUCalibration() // Let the ByPo stable in the first 2 seconds to measure the average value
{
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    GyroConfiguration();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

void RateCorrection()
{
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
}
void GyroRead()
{
  GyroConfiguration();
  RateCorrection();
}
void MPU_setup()
{
  Wire.begin();
  Wire.setClock(400000); // Set the I2C clock to 400kHz
  PowerManagementMPU();  // Wake up the MPU9250
  MPUCalibration();      // Calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
}

void GyroPrint()
{
  Serial.print(" RateRoll: ");
  Serial.print(RateRoll);
  Serial.print(" RatePitch: ");
  Serial.print(RatePitch);
  Serial.print(" Z: ");
  Serial.println(RateYaw);
}
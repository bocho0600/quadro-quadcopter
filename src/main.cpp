/* A ByPo Quadcopter Robot project of Kelvin Le
Contact: 
- Email: nguyenleminh1002@gmail.com
         minhnguyen.le@qut.edu.au
- Linkedin: https://www.linkedin.com/in/kelvin-le-1002/   */


#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "MPUinit.h"

void setup()
{

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set the I2C clock to 400kHz
  delay(250);            // Wait for the MPU9250 to power up
  PowerManagementMPU();  // Wake up the MPU9250
  MPUCalibration();      // Calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  GyroConfiguration();    // Configure the gyroscope
  RateCorrection();       // Correct the gyroscope data by calibration data
  Serial.print("Roll: "); // x-axis
  Serial.print(RateRoll);
  Serial.print(" Pitch: "); // y-axis
  Serial.print(RatePitch);
  Serial.print(" Yaw: "); // z-axis
  Serial.println(RateYaw);
  if ((RatePitch < -2) || (RatePitch > 2) || (RateRoll < -2) || (RateRoll > 2) || (RateYaw < -2) || (RateYaw > 2))
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

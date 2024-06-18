// Last update: 2021-07-29 20:00:00
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
  PowerManagementMPU(); // Wake up the MPU9250
}

void loop()
{
  GyroConfiguration(); // Configure the gyroscope
  Serial.print("Roll: ");   // x-axis
  Serial.print(RateRoll);
  Serial.print(" Pitch: "); // y-axis
  Serial.print(RatePitch);
  Serial.print(" Yaw: ");  // z-axis
  Serial.println(RateYaw);
}


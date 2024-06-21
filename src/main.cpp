/* A ByPo Quadcopter Robot project of Kelvin Le
Contact:
- Email: nguyenleminh1002@gmail.com
         minhnguyen.le@qut.edu.au
- Linkedin: https://www.linkedin.com/in/kelvin-le-1002/   */

#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "MPUinit.h"
#include "pwm.h"
#include "ControlSystem.h"

void setup()
{
  Serial.begin(115200);
  delay(250);  // Wait for the MPU9250 to power up
  MPU_setup(); // Setup the MPU , calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
  pwm_init();  // Initialize PWM for motors and buzzer
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  switch (state)
  {
  case CALIBRATION:
    break;
  case READY:
    buzzing(1); // Short beeps to indicate that the calibration is done
    state = RUN;
    break;
  case RUN:
    Serial.println(); // reset the line
    GyroRead();       // Read the gyroscope data
    GyroPrint();      // Print the gyroscope data

    if ((RatePitch < -2) || (RatePitch > 2) || (RateRoll < -2) || (RateRoll > 2) || (RateYaw < -2) || (RateYaw > 2))
    {
      digitalWrite(LED_BUILTIN, HIGH);
      // buzzing(2);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      // buzzing(0);
    }
    break;
  }
}

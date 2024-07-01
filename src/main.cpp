/* A ByPo Quadcopter Robot project of Kelvin Le
Contact:
- Email: nguyenleminh1002@gmail.com
         minhnguyen.le@qut.edu.au
- Linkedin: https://www.linkedin.com/in/kelvin-le-1002/   */

#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
#include "AngleInit.h"
#include "AccelInit.h"
#include "PIDcontrol.h"
uint32_t LoopTimer = 0;
void setup()
{
  Serial.begin(115200);
  delay(250);  // Wait for the MPU9250 to power up
  MPU_setup(); // Setup the MPU , calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
  pwm_init();  // Initialize PWM for motors and buzzer
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  buttonState = digitalRead(BUTTON_PIN); // read button state
  static int prev_buttonState = 0;
  if (buttonState == HIGH)
  {
    state = EMERGENCY;
  }
  prev_buttonState = buttonState;
  Serial.print(buttonState);
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
    AccelRead();      // Read the accelerometer data
    KalmanFilter(); // Kalman filter for the accelerometer data
    // GyroPrint();      // Print the gyroscope data
    // AccelPrint(2); // Print the accelerometer data
    
    PredictedAnglePrint();

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
  case EMERGENCY:
    motor_control(2048, 2048, 2048, 2048); //50% speed
    buzzing(3);

    break;
    delay(1);
  }
}
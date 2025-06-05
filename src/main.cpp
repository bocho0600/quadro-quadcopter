/* A ByPo Quadcopter Robot project of Kelvin Le
Contact:
- Email: nguyenleminh1002@gmail.com
         minhnguyen.le@qut.edu.au
- Linkedin: https://www.linkedin.com/in/kelvin-le-1002/ */

#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
#include "AngleInit.h"
#include "AccelInit.h"
#include "PIDcontrol.h"

uint32_t LoopTimer = 0;
// Constants
#define RXD2 16  // Serial2 RX (connects to Pi TX)
#define TXD2 17  // Serial2 TX (connects to Pi RX)

void setup()
{
  //Serial.begin(115200);
  Serial2.begin(230400, SERIAL_8N1, RXD2, TXD2); // UART for Raspberry Pi communication
  delay(250);  // Wait for the MPU9250 to power up
  MPU_setup(); // Setup the MPU , calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
  pwm_init();  // Initialize PWM for motors and buzzer
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void ParametersRead()
{
  //Serial.println(); // reset the line
  GyroRead();       // Read the gyroscope data
  AccelRead();      // Read the accelerometer data
  KalmanFilter();   // Kalman filter for the accelerometer data
}
void loop()
{
  ButtonCheck();
  ParametersRead();
  if ((RatePitch < -2) || (RatePitch > 2) || (RateRoll < -2) || (RateRoll > 2) || (RateYaw < -2) || (RateYaw > 2))
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // Serial.print(pb_falling);
  switch (state)
  {
  case CALIBRATION:
    buzzing(1); // Short beeps to indicate that the calibration is done
    digitalWrite(LED2, HIGH);
    state = READY;
    break;
  case READY:
    if (pb_falling)
    {
      buzzing(5);
      digitalWrite(LED2, LOW);  // Turn off RED LED
      digitalWrite(LED1, HIGH); // Turn on Blue LED
      motor_control(1000, 1000, 1000, 1000);
      state = RUN;
    }

    break;
  case RUN:

    // GyroPrint();      // Print the gyroscope data
    // AccelPrint(2); // Print the accelerometer data
    //PredictedAnglePrint();
    Serial2.println("<TEL," + String(KalmanAngleRoll) + "," + String(KalmanAnglePitch) + ">");

    motor_control(3000, 3000, 3000, 3000); // 50% speed
    if (pb_falling)
    {
      buzzing(5);
      digitalWrite(LED1, LOW);  // Turn off RED LED
      digitalWrite(LED2, HIGH); // Turn on Blue LED
      motor_control(1000, 1000, 1000, 1000);
      state = EMERGENCY;
    }
    break;
  case EMERGENCY:
    motor_control(2000, 2000, 2000, 2000); // 50% speed
    buzzing(4);
    break;
    delay(1);
  }
}
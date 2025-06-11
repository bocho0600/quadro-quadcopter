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
  Serial.begin(230400);
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

bool motor_armed = false;
float motor1_percent = 0;
float motor2_percent = 0;
float motor3_percent = 0;
float motor4_percent = 0;


String uartBuffer = "";

void readPiData() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '<') {
      uartBuffer = "";  // Start of new message
      uartBuffer += c;
    } else if (c == '>') {
      uartBuffer += c;
      // Serial.println(uartBuffer);  // Print entire message

      // Parse if message starts with <MOT,
      if (uartBuffer.startsWith("<MOT,")) {
        // Remove <MOT, and >
        String data = uartBuffer.substring(5, uartBuffer.length() - 1);
        int idx1 = data.indexOf(',');
        int idx2 = data.indexOf(',', idx1 + 1);
        int idx3 = data.indexOf(',', idx2 + 1);
        int idx4 = data.indexOf(',', idx3 + 1);

        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3) {
          motor_armed    = (data.substring(0, idx1).toInt() != 0); // true if not zero
          motor1_percent = data.substring(idx1 + 1, idx2).toFloat();
          motor2_percent = data.substring(idx2 + 1, idx3).toFloat();
          motor3_percent = data.substring(idx3 + 1, idx4).toFloat();
          motor4_percent = data.substring(idx4 + 1).toFloat();
        }
      }

      uartBuffer = "";  // Clear for next message
    } else {
      uartBuffer += c;
    }
  }
}


void loop()
{
  Serial2.print("<TEL," + String(KalmanAngleRoll, 2) + "," + String(KalmanAnglePitch, 2) + ">");
  readPiData(); // <-- Add this line
  ButtonCheck();
  //Serial.println(motor_armed);  // Print entire message

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
      //motor_control(1000, 1000, 1000, 1000);
      state = RUN;
    }

    break;
  case RUN:

    // GyroPrint();      // Print the gyroscope data
    // AccelPrint(2); // Print the accelerometer data
    //PredictedAnglePrint();
    
    
    MotorWrite(motor_armed, motor1_percent, motor2_percent, motor3_percent, motor4_percent); // Control motors based on input percentages
    //Serial.println(motor1_percent);
    if (pb_falling)
    {
      buzzing(5);
      digitalWrite(LED1, LOW);  // Turn off RED LED
      digitalWrite(LED2, HIGH); // Turn on Blue LED
     // motor_control(1000, 1000, 1000, 1000);
      state = EMERGENCY;
    }
    break;
  case EMERGENCY:
    //motor_control(2000, 2000, 2000, 2000); // 50% speed
    buzzing(4);
    break;
    delay(1);
  }
}
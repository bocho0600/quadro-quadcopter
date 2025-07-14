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
#include "controller.h"
uint32_t LoopTimer = 0;
// Constants
#define RXD2 16  // Serial2 RX (connects to Pi TX)
#define TXD2 17  // Serial2 TX (connects to Pi RX)
#define ARM_PIN 32 // Pin to read the arm state


volatile unsigned long throttleRise = 0;
volatile uint16_t throttlePulse = 1000;

volatile unsigned long throttleRiseTime = 0;
volatile uint16_t throttlePulseWidth = 1000;  // Default to minimum

// Interrupt handler for throttle pin
void IRAM_ATTR throttleInterrupt() {
  if (digitalRead(THROTTLE_PIN)) {
    throttleRiseTime = micros();
  } else {
    throttlePulseWidth = micros() - throttleRiseTime;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(230400, SERIAL_8N1, RXD2, TXD2); // UART for Raspberry Pi communication

  delay(250);  // Wait for the MPU9250 to power up
  MPU_setup(); // Setup the MPU , calibrate the gyroscope, let the ByPo stable in the first 2 seconds to measure the average value
  pwm_init();  // Initialize PWM for motors and buzzer

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup throttle pin interrupt
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ARM_PIN, INPUT); // Arm pin with pull-up resistor
  // attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleInterrupt, CHANGE);
}

void ParametersRead()
{
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
float input_arm = 0;

void readPiData() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '<') {
      uartBuffer = "";  // Start of new message
      uartBuffer += c;
    } else if (c == '>') {
      uartBuffer += c;

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
  readPiData();
  ButtonCheck();
  ParametersRead();
  ControllerRead();   // Update desired rates and errors

  motor_armed = ArmRead(); // Read the arm state

  // Debug LED if rates exceed limits
  if ((RatePitch < -2) || (RatePitch > 2) || (RateRoll < -2) || (RateRoll > 2) || (RateYaw < -2) || (RateYaw > 2)) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  switch (state)
  {
    case CALIBRATION:
      buzzing(1); // Short beeps to indicate that the calibration is done
      digitalWrite(LED2, HIGH);
      state = READY;
      break;

    case READY:
      Serial.print("Arm State: ");
      Serial.println(motor_armed);
      if (pb_falling)
      {
        buzzing(5);
        digitalWrite(LED2, LOW);  // Turn off RED LED
        digitalWrite(LED1, HIGH); // Turn on Blue LED
        PIDReset();
        MotorConnect(1);
        state = RUN;
        
      }
      break;

    case RUN:
    {
      // Get throttle from PWM (CH3)
      
      // Serial.println(InputThrottle);
      // GyroPrint(); // Print the predicted angles

      PID_Execution();    // Run the PID controller, updates InputRoll, InputPitch, InputYaw, InputThrottle
      MotorInput(motor_armed);       // Calculate motor outputs based on PID results

      if (pb_falling)
      {
        buzzing(5);
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        state = EMERGENCY;
      }
      break;
    }

    case EMERGENCY:
      buzzing(4);
      break;
      delay(1);
  }
}

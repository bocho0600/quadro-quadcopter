#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "MPUinit.h"
#include "pwm.h"

// Initialize PWM for motors and buzzer
void pwm_init()
{
      // Buzzer setup on channel 0, 3000 Hz, 8-bit resolution
      ledcSetup(0, 3000, 8);
      ledcAttachPin(BUZZER_PIN, 0);

      // Motor setup on channels 1-4, 5000 Hz, 8-bit resolution
      for (int i = 1; i <= 4; i++)
      {
            ledcSetup(i, 5000, 8);
      }
      ledcAttachPin(MOTOR1, 1);
      ledcAttachPin(MOTOR2, 2);
      ledcAttachPin(MOTOR3, 3);
      ledcAttachPin(MOTOR4, 4);
}

// Function to control motor speed
void motor_control(uint8_t speed1, uint8_t speed2, uint8_t speed3, uint8_t speed4)
{
      speed1 = constrain(speed1, 0, 255);
      speed2 = constrain(speed2, 0, 255);
      speed3 = constrain(speed3, 0, 255);
      speed4 = constrain(speed4, 0, 255);
      Serial.print(" Speed1: ");
      Serial.print(speed1);
      Serial.print(" Speed2: ");
      Serial.print(speed2);
      Serial.print(" Speed3: ");
      Serial.print(speed3);
      Serial.print(" Speed4: ");
      ledcWrite(1, speed1);
      ledcWrite(2, speed2);
      ledcWrite(3, speed3);
      ledcWrite(4, speed4); // Write the speed to the motor by setting the duty cycle of PWM
}

// Function to control buzzer
void buzzing(uint8_t type)
{
      static uint8_t prev_type;
      if (type == 1)
      {
            // Pattern 1: Short beeps
            ledcSetup(0, 3000, 8);
            for (int i = 0; i < 2; i++)
            {
                  ledcWrite(0, 200); // Turn on buzzer
                  delay(100);        // Wait
                  ledcWrite(0, 0);   // Turn off buzzer
                  delay(100);        // Wait
            }
      }
      else if (type == 2)
      {
            // Pattern 2: Continuous buzzing
            if (type != prev_type)
            {
                  ledcSetup(0, 2500, 8);
            }

            ledcWrite(0, 200); // Turn on buzzer
      }
      else if (type == 0)
      {
            // Turn off buzzer
            ledcWrite(0, 0);
      }
      prev_type = type;
}

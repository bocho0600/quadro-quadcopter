#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
bool buttonState = 0;
bool pb_falling = 0;
bool pb_rising = 0;
void ButtonCheck()
{
      bool prev_buttonState = buttonState;
      buttonState = digitalRead(BUTTON_PIN); // read button state
      bool buttonChanged = (buttonState ^ prev_buttonState);
      pb_rising = buttonChanged & ~buttonState;
      pb_falling = buttonChanged & buttonState;
      if (pb_falling)
      {
            ledcSetup(0, 2000, 8);
            ledcWrite(0, 200); // Turn on buzzer with 50% volume when pressing button
            delay(50);
            ledcWrite(0, 0); // Turn off buzzer
      }
}
// Initialize PWM for motors and buzzer
const int freq = 50;       // Frequency in Hz for the ESC signal
const int pwmChannel = 0;  // PWM channel (0-15)
const int resolution = 12; // PWM resolution
void pwm_init()
{
      // Buzzer setup on channel 0, 3000 Hz, 8-bit resolution
      ledcSetup(0, 3000, 8);
      ledcAttachPin(BUZZER_PIN, 0);
      // Motor setup on channels 1-4, 250 Hz, 12-bit resolution
      ledcSetup(2, freq, 16); // 250 Hz, 12-bit resolution
      ledcSetup(3, freq, 16); // 250 Hz, 12-bit resolution
      ledcSetup(4, freq, 16); // 250 Hz, 12-bit resolution
      ledcSetup(5, freq, 16); // 250 Hz, 12-bit resolution
      ledcAttachPin(MOTOR1, 5);
      ledcAttachPin(MOTOR2, 2);
      ledcAttachPin(MOTOR3, 3);
      ledcAttachPin(MOTOR4, 4);
}

// Function to control motor speed
void motor_control(int speed1, int speed2, int speed3, int speed4)
{
      speed1 = map(speed1, 1000, 2000, 3277, 6554);
      speed2 = map(speed2, 1000, 2000, 3277, 6554);
      speed3 = map(speed3, 1000, 2000, 3277, 6554);
      speed4 = map(speed4, 1000, 2000, 3277, 6554);
      // speed1 = constrain(speed1, 0, 4096); //
      // speed2 = constrain(speed2, 0, 4096);
      // speed3 = constrain(speed3, 0, 4096);
      // speed4 = constrain(speed4, 0, 4096);
      // Serial.print(" Speed1: ");
      // Serial.print(speed1);
      // Serial.print(" Speed2: ");
      // Serial.print(speed2);
      // Serial.print(" Speed3: ");
      // Serial.print(speed3);
      // Serial.print(" Speed4: ");
      ledcWrite(5, speed1);
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
                  ledcWrite(0, 255 >> 1); // Turn on buzzer
                  digitalWrite(LED_BUILTIN, HIGH);
                  delay(100);      // Wait
                  ledcWrite(0, 0); // Turn off buzzer
                  digitalWrite(LED_BUILTIN, LOW);
                  delay(100); // Wait
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
      else if (type == 3)
      {
            // ledcWrite(0, 200); // Turn on buzzer
            ledcWrite(0, 0); // Turn on buzzer
            while (1)
            {
                  buttonState = digitalRead(BUTTON_PIN); // read button state
                  // Serial.println(buttonState);
                  ledcSetup(0, 2000, 8);
                  ledcWrite(0, 200); // Turn on buzzer
                  digitalWrite(LED1, HIGH);
                  digitalWrite(LED2, LOW);
                  digitalWrite(LED_BUILTIN, HIGH);
                  delay(300); // Wait
                  ledcSetup(0, 3000, 8);
                  digitalWrite(LED_BUILTIN, LOW);
                  digitalWrite(LED1, LOW);
                  digitalWrite(LED2, HIGH);
                  delay(300); // Wait
                  Serial.println("EMERGENCY STOP!");
            }
      }
      else if (type == 4)
      {
            while (1)
            {
                  buttonState = digitalRead(BUTTON_PIN); // read button state
                  // Serial.println(buttonState);
                  digitalWrite(LED1, HIGH);
                  digitalWrite(LED2, LOW);
                  digitalWrite(LED_BUILTIN, HIGH);
                  delay(300); // Wait
                  digitalWrite(LED_BUILTIN, LOW);
                  digitalWrite(LED1, LOW);
                  digitalWrite(LED2, HIGH);
                  delay(300); // Wait
                  Serial.println("EMERGENCY STOP!");
            }
      }
      else if (type == 5)
      {
            ledcSetup(0, 2000, 8);
            for (int i = 0; i < 3; i++)
            {
                  delay(1000);
                  ledcWrite(0, 200); // Turn on buzzer
                  digitalWrite(LED1, HIGH);
                  delay(100);
                  ledcWrite(0, 0); // Turn off buzzer
                  digitalWrite(LED1, LOW);
            }
            delay(1000);
            ledcWrite(0, 200); // Turn on buzzer
            delay(100);
            ledcWrite(0, 0); // Turn off buzzer
            ledcSetup(0, 3000, 8);
            ledcWrite(0, 200); // Turn on buzzer
            delay(200);
            ledcWrite(0, 0); // Turn off buzzer
            delay(100);
      }
      prev_type = type;
}

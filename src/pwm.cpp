#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
#include <ESP32Servo.h>

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


// Define your ESC control pins here
const int ESC_PIN1 = 5;
const int ESC_PIN2 = 2;
const int ESC_PIN3 = 3;
const int ESC_PIN4 = 4;
Servo esc1, esc2, esc3, esc4;

void pwm_init()
{
  esc1.attach(ESC_PIN1);
  esc2.attach(ESC_PIN2);
  esc3.attach(ESC_PIN3);
  esc4.attach(ESC_PIN4);

}

// Function to control motor speed
void motor_control(int percent1, int percent2, int percent3, int percent4) {
  // Constrain percent inputs to 0-100%
  percent1 = constrain(percent1, 0, 100);
  percent2 = constrain(percent2, 0, 100);
  percent3 = constrain(percent3, 0, 100);
  percent4 = constrain(percent4, 0, 100);

  // Map percent to pulse width in microseconds (1000–2000us)
  int pwm1 = map(percent1, 0, 100, 1000, 2000);
  int pwm2 = map(percent2, 0, 100, 1000, 2000);
  int pwm3 = map(percent3, 0, 100, 1000, 2000);
  int pwm4 = map(percent4, 0, 100, 1000, 2000);

  // Send PWM signals to ESCs
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);
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

#ifndef PWM_H
#define PWM_H
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#define MOTOR1 13
#define MOTOR2 12
#define MOTOR3 14
#define MOTOR4 27
#define BUZZER_PIN 18
void pwm_init();
void motor_control(uint8_t spped1, uint8_t speed2, uint8_t speed3, uint8_t speed4);
void buzzing(uint8_t type);

#endif
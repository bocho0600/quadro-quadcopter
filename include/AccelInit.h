#ifndef ACCELINIT_H
#define ACCELINIT_H
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
extern float AccX, AccY, AccZ; // Accelerometer data
extern float AngleRoll, AnglePitch; // Accelerometer angles
void AccelConfiguration();
void AccelResponse();
void AccelRead();
void AccelPrint(uint8_t type);
#endif
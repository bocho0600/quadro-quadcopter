#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <Arduino.h>
#include <Wire.h>
#define ARM_PIN 32 // Pin to read the arm state
// float InputRoll, InputThrottle, InputPitch, InputYaw;
bool ArmRead();
#endif
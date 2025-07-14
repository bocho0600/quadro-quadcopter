#ifndef PIDCONTROL_H
#define PIDCONTROL_H
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#define THROTTLE_PIN 33  // CH3 from HotRC receiver (Throttle)
extern float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
extern float InputRoll, InputThrottle, InputPitch, InputYaw;
extern float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
extern float PrevIntermRateRoll, PrevIntermRatePitch, PrevIntermRateYaw;
extern float PIDReturn[];
extern float PRateRoll, PRatePitch, PRateYaw;
extern float IRateRoll, IRatePitch, IRateYaw;
extern float DRateRoll, DRatePitch, DRateYaw;
extern float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void PIDReset();
void ControllerRead();
void PID_Execution();
void MotorInput(bool armed);

#endif
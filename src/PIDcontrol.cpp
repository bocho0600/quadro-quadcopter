#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "pwm.h"
#include "AngleInit.h"
#include "AccelInit.h"
#include "PIDcontrol.h"

/* PID control parameters */
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevIntermRateRoll, PrevIntermRatePitch, PrevIntermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.6, PRatePitch = 0.6, PRateYaw = 2;
float IRateRoll = 3.5, IRatePitch = 3.5, IRateYaw = 12;
float DRateRoll = 0.03, DRatePitch = 0.03, DRateYaw = 0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float Ts = 0.004;
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
      float Pterm = P * Error;
      float Iterm = PrevIterm + I * (Error + PrevError) * Ts / 2;
      if (Iterm > 400)
            Iterm = 400;     // Limit the integral term to 400
      else if (Iterm < -400) // Limit the integral term to -400
            Iterm = -400;
      float Dterm = D * (Error - PrevError) / Ts;
      float PIDOutput = Pterm + Iterm + Dterm;
      if (PIDOutput > 400)
            PIDOutput = 400; // Limit the output to 400
      else if (PIDOutput < -400)
            PIDOutput = -400; // Limit the output to -400
      PIDReturn[0] = PIDOutput;
      PIDReturn[1] = Error;
      PIDReturn[2] = Iterm;
}
void PIDReset()
{
      PrevErrorRateRoll = 0;
      PrevErrorRatePitch = 0;
      PrevErrorRateYaw = 0;
      PrevIntermRateRoll = 0;
      PrevIntermRatePitch = 0;
      PrevIntermRateYaw = 0;
}

void ControllerRead()
{
      // Capturing the desired rates from the controller
      DesiredRateRoll = 0;
      DesiredRatePitch = 0;
      InputThrottle = 0;
      InputYaw = 0;

      // Calculating the error rates from the current state
      ErrorRateRoll = DesiredRateRoll - RateRoll;
      ErrorRatePitch = DesiredRatePitch - RatePitch;
      ErrorRateYaw = DesiredRateYaw - RateYaw;
}

void PID_Execution()
{
      pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevIntermRateRoll);
      InputRoll = PIDReturn[0];
      PrevErrorRateRoll = PIDReturn[1];
      PrevIntermRateRoll = PIDReturn[2];
      pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevIntermRatePitch);
      InputPitch = PIDReturn[0];
      PrevErrorRatePitch = PIDReturn[1];
      PrevIntermRatePitch = PIDReturn[2];
      pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevIntermRateYaw);
      InputYaw = PIDReturn[0];
      PrevErrorRateYaw = PIDReturn[1];
      PrevIntermRateYaw = PIDReturn[2];
      if (InputThrottle > 1800)
            InputThrottle = 1800;
      else if (InputThrottle < 1200)
            InputThrottle = 1200;
}
void MotorInput()
{

      MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw) * 1.024;
      MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw) * 1.024;
      MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw) * 1.024;
      MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw) * 1.024;
      if (MotorInput1 > 2000)
            MotorInput1 = 1999;
      if (MotorInput2 > 2000)
            MotorInput2 = 1999;
      if (MotorInput3 > 2000)
            MotorInput3 = 1999;
      if (MotorInput4 > 2000)
            MotorInput4 = 1999;
}
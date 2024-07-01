#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "GyroInit.h"
#include "AccelInit.h"
#include "pwm.h"
#include "AngleInit.h"

/* The value range of Roll Pitch Yaw is from -500 to 500 */
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4; // 2*2 = 4
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[2] = {0, 0}; // angle prediction and uncertainty of the prediction

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    // Prediction step
    KalmanState += 0.004 * KalmanInput;      // Predict the current state
    KalmanUncertainty += 0.004 * 0.004 * 16; // Add process noise, Q = (0.004 * 4)^2

    // Update step
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // Measurement noise R = 3*3 = 9
    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);  // Correct the state with measurement
    KalmanUncertainty *= (1 - KalmanGain);                          // Correct the uncertainty

    // Store the results
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void KalmanFilter()
{
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}

void PredictedAnglePrint()
{
    Serial.print("Predicted Roll: ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Predicted Pitch: ");
    Serial.println(KalmanAnglePitch);
}
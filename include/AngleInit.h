#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[2];
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void KalmanFilter();
void PredictedAnglePrint();
#endif
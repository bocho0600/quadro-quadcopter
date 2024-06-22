#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll; // 2*2 = 4
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[2]; // angle prediction and uncertainty of the prediction
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void KalmanFilter();
void PredictedAnglePrint();
#endif
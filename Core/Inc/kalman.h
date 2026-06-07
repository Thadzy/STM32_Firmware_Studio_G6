#ifndef __KALMAN_H
#define __KALMAN_H

/* 4-state Physical DC Motor Kalman filter.
   States : x = [pos_rad, vel_rads, tau_Nm, i_a_Amps]
   Measurement : encoder position (rad) at 1 kHz.
   Input : Motor voltage (V)
   Called every 1 ms from MotorCtrl_Tick1kHz() in the TIM6 ISR.              */

typedef struct {
    float x[4];      /* [pos_rad, vel_rads, tau_Nm, i_a_Amps]                */
    float P[4][4];   /* error covariance                                     */
} KalmanState_t;

void Kalman_Init(KalmanState_t *k, float pos0_rad);
void Kalman_Update(KalmanState_t *k, float pos_meas_rad, float voltage_v);

#endif /* __KALMAN_H */

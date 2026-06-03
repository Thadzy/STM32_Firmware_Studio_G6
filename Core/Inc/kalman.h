#ifndef __KALMAN_H
#define __KALMAN_H

/* 4-state kinematic Kalman filter.
   States : x = [pos_rad, vel_rads, acc_rads2, jerk_rads3]
   Measurement : encoder position (rad) at 1 kHz.
   Called every 1 ms from MotorCtrl_Tick1kHz() in the TIM6 ISR.              */

typedef struct {
    float x[4];      /* [pos_rad, vel_rads, acc_rads2, jerk_rads3]           */
    float P[4][4];   /* error covariance                                     */
} KalmanState_t;

void Kalman_Init(KalmanState_t *k, float pos0_rad);
void Kalman_Update(KalmanState_t *k, float pos_meas_rad);

#endif /* __KALMAN_H */

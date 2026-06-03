#include "kalman.h"
#include "params.h"
#include <string.h>

/* State-transition matrix F (constant, dt = KALMAN_DT = 0.001 s):
   [ 1  dt  dt²/2  dt³/6 ]
   [ 0   1     dt  dt²/2 ]
   [ 0   0      1     dt ]
   [ 0   0      0      1 ]                                                    */
#define KF_DT   KALMAN_DT
#define KF_DT2  (KF_DT * KF_DT * 0.5f)
#define KF_DT3  (KF_DT * KF_DT * KF_DT * (1.0f / 6.0f))

void Kalman_Init(KalmanState_t *k, float pos0_rad)
{
    memset(k, 0, sizeof(*k));
    k->x[0] = pos0_rad;
    k->P[0][0] = 0.01f;     /* pos  uncertainty: ~0.1 rad                   */
    k->P[1][1] = 10.0f;     /* vel  uncertainty: ~3.2 rad/s                 */
    k->P[2][2] = 100.0f;    /* acc  uncertainty: ~10 rad/s²                 */
    k->P[3][3] = 1000.0f;   /* jerk uncertainty: ~31.6 rad/s³               */
}

void Kalman_Update(KalmanState_t *k, float pos_meas_rad)
{
    float *x  = k->x;
    float (*P)[4] = k->P;

    /* ----------------------------------------------------------------
       PREDICT: x = F * x
       ---------------------------------------------------------------- */
    float x0 = x[0] + KF_DT*x[1] + KF_DT2*x[2] + KF_DT3*x[3];
    float x1 = x[1] + KF_DT*x[2] + KF_DT2*x[3];
    float x2 = x[2] + KF_DT*x[3];
    float x3 = x[3];
    x[0] = x0; x[1] = x1; x[2] = x2; x[3] = x3;

    /* ----------------------------------------------------------------
       PREDICT: P = F * P * F' + Q
       Step 1: FP = F * P (exploit F upper-triangular structure)
       ---------------------------------------------------------------- */
    float FP[4][4];
    for (int j = 0; j < 4; j++) {
        FP[0][j] = P[0][j] + KF_DT*P[1][j] + KF_DT2*P[2][j] + KF_DT3*P[3][j];
        FP[1][j] = P[1][j] + KF_DT*P[2][j] + KF_DT2*P[3][j];
        FP[2][j] = P[2][j] + KF_DT*P[3][j];
        FP[3][j] = P[3][j];
    }
    /* Step 2: P = FP * F' (F' lower-triangular) */
    for (int i = 0; i < 4; i++) {
        P[i][0] = FP[i][0];
        P[i][1] = FP[i][0]*KF_DT  + FP[i][1];
        P[i][2] = FP[i][0]*KF_DT2 + FP[i][1]*KF_DT  + FP[i][2];
        P[i][3] = FP[i][0]*KF_DT3 + FP[i][1]*KF_DT2 + FP[i][2]*KF_DT + FP[i][3];
    }
    /* Add process noise Q (diagonal) */
    P[0][0] += KALMAN_Q_POS;
    P[1][1] += KALMAN_Q_VEL;
    P[2][2] += KALMAN_Q_ACC;
    P[3][3] += KALMAN_Q_JERK;

    /* ----------------------------------------------------------------
       UPDATE: scalar measurement z = pos_meas_rad, H = [1 0 0 0]
       ---------------------------------------------------------------- */
    float y    = pos_meas_rad - x[0];           /* innovation               */
    float S    = P[0][0] + KALMAN_R_POS;        /* innovation covariance    */
    float Sinv = 1.0f / S;

    /* Kalman gain K = P * H' / S  (H=[1,0,0,0] → K = first col of P / S)  */
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = P[i][0] * Sinv;

    /* State update: x += K * y */
    for (int i = 0; i < 4; i++) x[i] += K[i] * y;

    /* Covariance update: P = (I - K*H)*P
       Save the original first row before in-place update.                   */
    float P0[4];
    for (int j = 0; j < 4; j++) P0[j] = P[0][j];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] -= K[i] * P0[j];
        }
    }
}

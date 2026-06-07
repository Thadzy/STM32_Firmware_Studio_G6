#include "kalman.h"
#include "params.h"
#include "system_state.h"
#include <string.h>

/* Exact ZOH Discrete State-Transition Matrix Phi (dt = 0.001) */
static const float PHI[4][4] = {
    {1.0f,          0.0009989989f, -6.8679895e-7f,  1.2007346e-6f},
    {0.0f,          0.997317666f,  -0.0013729679f,  0.0020608558f},
    {0.0f,          0.0f,           1.0f,           0.0f},
    {0.0f,         -1.2692052f,     0.0010163110f,  0.36477577f}
};

/* Exact ZOH Discrete Input Matrix Gamma (dt = 0.001) */
static const float GAMMA[4] = {
    2.97955747e-7f,
    8.29236580e-4f,
    0.0f,
    0.435397404f
};

void Kalman_Init(KalmanState_t *k, float pos0_rad)
{
    memset(k, 0, sizeof(*k));
    k->x[0] = pos0_rad;
    k->P[0][0] = 0.01f;     /* pos  uncertainty: ~0.1 rad                   */
    k->P[1][1] = 10.0f;     /* vel  uncertainty: ~3.2 rad/s                 */
    k->P[2][2] = 1.0f;      /* tau  uncertainty: ~1.0 Nm                    */
    k->P[3][3] = 1.0f;      /* ia   uncertainty: ~1.0 A                     */
}

void Kalman_Update(KalmanState_t *k, float pos_meas_rad, float voltage_v)
{
    float *x  = k->x;
    float (*P)[4] = k->P;

    /* ----------------------------------------------------------------
       PREDICT: x = Phi * x + Gamma * u
       ---------------------------------------------------------------- */
    float x_pred[4];
    for (int i = 0; i < 4; i++) {
        x_pred[i] = GAMMA[i] * voltage_v;
        for (int j = 0; j < 4; j++) {
            x_pred[i] += PHI[i][j] * x[j];
        }
    }

    /* ----------------------------------------------------------------
       PREDICT: P = Phi * P * Phi' + Q
       Step 1: PhiP = Phi * P
       ---------------------------------------------------------------- */
    float PhiP[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            PhiP[i][j] = 0.0f;
            for (int l = 0; l < 4; l++) {
                PhiP[i][j] += PHI[i][l] * P[l][j];
            }
        }
    }

    /* Step 2: P_pred = PhiP * Phi' */
    float P_pred[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_pred[i][j] = 0.0f;
            for (int l = 0; l < 4; l++) {
                P_pred[i][j] += PhiP[i][l] * PHI[j][l]; /* Phi[j][l] is Phi^T[l][j] */
            }
        }
    }

    /* Add process noise Q (diagonal). We map the existing kinematic tuning
       parameters to our new states to maintain dashboard compatibility. */
    P_pred[0][0] += TuningParams.kalman.q_pos;   /* Position Q */
    P_pred[1][1] += TuningParams.kalman.q_vel;   /* Velocity Q */
    P_pred[2][2] += TuningParams.kalman.q_acc;   /* Disturbance Torque Q */
    P_pred[3][3] += TuningParams.kalman.q_jerk;  /* Armature Current Q */

    /* ----------------------------------------------------------------
       UPDATE: scalar measurement z = pos_meas_rad, H = [1 0 0 0]
       y = z - H * x_pred = pos_meas_rad - x_pred[0]
       ---------------------------------------------------------------- */
    float y    = pos_meas_rad - x_pred[0];           /* innovation               */
    float S    = P_pred[0][0] + TuningParams.kalman.r_pos; /* innovation covariance  */
    float Sinv = 1.0f / S;

    /* Kalman gain K = P_pred * H' / S  (H=[1,0,0,0] -> K = first col of P_pred / S) */
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = P_pred[i][0] * Sinv;

    /* State update: x = x_pred + K * y */
    for (int i = 0; i < 4; i++) x[i] = x_pred[i] + K[i] * y;

    /* Covariance update: P = (I - K*H) * P_pred
       Since H = [1,0,0,0], the matrix (I - K*H) just subtracts K[i] from the first column of I.
       So P[i][j] = P_pred[i][j] - K[i]*P_pred[0][j] */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = P_pred[i][j] - K[i] * P_pred[0][j];
        }
    }
}

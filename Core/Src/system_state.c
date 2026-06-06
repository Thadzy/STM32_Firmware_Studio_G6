#include "system_state.h"
#include "params.h"

RobotState_t RobotState = {0};

TuningParams_t TuningParams = {
    .pos_pid = {PID_POS_KP, PID_POS_KI, PID_POS_KD},
    .spd_pid = {PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD},
    .scurve  = {SCURVE_VMAX_RADS, SCURVE_AMAX_RADS2, SCURVE_JMAX_RADS3},
    .ff      = {FF_VELOCITY, FF_ACCEL, FF_DISTURBANCE},
    .kalman  = {KALMAN_Q_POS, KALMAN_Q_VEL, KALMAN_Q_ACC, KALMAN_Q_JERK, KALMAN_R_POS},
    .zvd     = {false}
};

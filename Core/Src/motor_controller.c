#include "motor_controller.h"
#include "kalman.h"
#include "hw_io.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include <math.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;

/* -------------------------------------------------------------------------
   Unit conversion
   ------------------------------------------------------------------------- */
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif
#define COUNTS_TO_RAD   (2.0f * M_PI / (float)ENCODER_CPR)
#define RAD_TO_COUNTS   ((float)ENCODER_CPR / (2.0f * M_PI))
#define DT_INNER        KALMAN_DT                       /* 1 ms              */
#define DT_OUTER        (1.0f / (float)OUTER_LOOP_HZ)  /* 10 ms             */

/* -------------------------------------------------------------------------
   S-curve trajectory state
   ------------------------------------------------------------------------- */
typedef struct {
    float pos;
    float vel;
    float acc;
    float target;
    bool  done;
} SCurvePlan_t;

/* -------------------------------------------------------------------------
   Module-private state
   ------------------------------------------------------------------------- */
static KalmanState_t s_kalman;
static int32_t       s_pos_counts;
static uint16_t      s_last_enc;

/* Velocity setpoint passed from outer loop to inner loop */
static volatile float s_vel_sp;
static volatile float s_acc_sp;

/* Homing creep mode — bypasses S-curve and position PID */
static volatile bool  s_homing_mode;
static volatile float s_homing_vel;

/* Speed PID state */
static float s_spd_integral;
static float s_spd_prev_err;

/* Position PID state */
static float s_pos_integral;
static float s_pos_prev_meas;

/* S-curve and ZVD */
static SCurvePlan_t s_sc;
static float        s_zvd_buf[ZVD_BUF_SIZE];
static uint8_t      s_zvd_head;
static uint16_t     s_settle_ticks;

static bool s_running;
static bool s_zvd_bypass;  /* true during homing — no rod, no need for input shaping */

/* -------------------------------------------------------------------------
   S-curve helpers
   ------------------------------------------------------------------------- */

/* Conservative stopping distance from current (vel, acc) in direction of motion.
   Uses the S-curve braking profile: jerk-down phase + deceleration phase.   */
static float scurve_stop_dist(float vel, float acc)
{
    if (vel <= 0.0f) return 0.0f;

    /* Phase A: reduce acc to 0 via -Jmax if acc is positive */
    float t_a = (acc > 0.0f) ? acc / SCURVE_JMAX_RADS3 : 0.0f;
    float v_a = vel + acc * t_a - 0.5f * SCURVE_JMAX_RADS3 * t_a * t_a;
    float d_a = vel * t_a + 0.5f * acc * t_a * t_a
                - (1.0f / 6.0f) * SCURVE_JMAX_RADS3 * t_a * t_a * t_a;

    /* Phase B+C: decelerate v_a to 0 (conservative — ignores jerk-limited
       nature of decel, compensated by 1.5× safety factor)                   */
    float d_bc = (v_a > 0.0f)
                 ? v_a * v_a / (2.0f * SCURVE_AMAX_RADS2) * 1.5f
                 : 0.0f;

    return d_a + d_bc;
}

static void scurve_step(float dt)
{
    if (s_sc.done) return;

    float dist = s_sc.target - s_sc.pos;

    /* Done if very close and nearly stopped */
    if (fabsf(dist) < 0.001f && fabsf(s_sc.vel) < 0.01f) {
        s_sc.pos  = s_sc.target;
        s_sc.vel  = 0.0f;
        s_sc.acc  = 0.0f;
        s_sc.done = true;
        return;
    }

    float dir     = (dist >= 0.0f) ? 1.0f : -1.0f;
    float vel_dir = s_sc.vel * dir;
    float acc_dir = s_sc.acc * dir;
    float abs_dist = fabsf(dist);

    float stop_dist = scurve_stop_dist(vel_dir, acc_dir);

    float jerk;
    if (abs_dist <= stop_dist) {
        /* Braking zone */
        jerk = -SCURVE_JMAX_RADS3 * dir;
    } else if (vel_dir >= SCURVE_VMAX_RADS - 0.01f) {
        /* At cruise velocity — bring acc to 0 */
        jerk = (acc_dir > 0.01f) ? -SCURVE_JMAX_RADS3 * dir : 0.0f;
    } else if (acc_dir < SCURVE_AMAX_RADS2 - 0.01f) {
        /* Still below Amax — accelerate */
        jerk = SCURVE_JMAX_RADS3 * dir;
    } else {
        /* Holding Amax */
        jerk = 0.0f;
    }

    s_sc.acc += jerk * dt;
    if (s_sc.acc >  SCURVE_AMAX_RADS2) s_sc.acc =  SCURVE_AMAX_RADS2;
    if (s_sc.acc < -SCURVE_AMAX_RADS2) s_sc.acc = -SCURVE_AMAX_RADS2;

    s_sc.vel += s_sc.acc * dt;
    if (s_sc.vel >  SCURVE_VMAX_RADS) s_sc.vel =  SCURVE_VMAX_RADS;
    if (s_sc.vel < -SCURVE_VMAX_RADS) s_sc.vel = -SCURVE_VMAX_RADS;

    s_sc.pos += s_sc.vel * dt;
}

/* -------------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------------- */

void MotorCtrl_Init(void)
{
    /* Zero all state */
    memset(&s_kalman,   0, sizeof(s_kalman));
    memset(&s_sc,       0, sizeof(s_sc));
    memset(s_zvd_buf,   0, sizeof(s_zvd_buf));
    s_pos_counts   = 0;
    s_vel_sp       = 0.0f;
    s_acc_sp       = 0.0f;
    s_spd_integral = 0.0f;
    s_spd_prev_err = 0.0f;
    s_pos_integral = 0.0f;
    s_pos_prev_meas = 0.0f;
    s_zvd_head     = 0;
    s_settle_ticks = 0;
    s_running      = false;

    /* Capture current encoder position as position 0 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    s_last_enc = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    Kalman_Init(&s_kalman, 0.0f);
}

void MotorCtrl_SetTarget(float target_rad)
{
    s_homing_mode  = false;
    s_sc.target    = target_rad;
    s_sc.done      = false;
    s_settle_ticks = 0;
    s_running      = true;
}

void MotorCtrl_Stop(void)
{
    s_vel_sp   = 0.0f;
    s_acc_sp   = 0.0f;
    s_running  = false;
    Motor_SetPWM(0);
}

bool MotorCtrl_IsAtTarget(void)
{
    return s_sc.done
        && s_settle_ticks >= (uint16_t)(ZVD_T3_STEPS + 10u)
        && fabsf(s_kalman.x[0] - s_sc.target) < POSITION_DEADBAND_RAD
        && fabsf(s_kalman.x[1]) < VELOCITY_SETTLED_RADS;
}

float MotorCtrl_GetPosition_rad(void)
{
    return s_kalman.x[0];
}

void MotorCtrl_HomingCreep(int8_t dir)
{
    s_homing_mode = true;
    s_homing_vel  = (dir >= 0) ? HOMING_VEL_RADS : -HOMING_VEL_RADS;
    s_running     = true;
}

bool MotorCtrl_IsAtPosition(void)
{
    /* Like IsAtTarget but without the velocity gate.
       When ZVD is bypassed (homing, no rod) only a short settle is needed.
       When ZVD is active the full tail (T3+10 ticks) must flush first.    */
    uint16_t need = s_zvd_bypass ? 5u : (uint16_t)(ZVD_T3_STEPS + 10u);
    return s_sc.done
        && s_settle_ticks >= need
        && fabsf(s_kalman.x[0] - s_sc.target) < POSITION_DEADBAND_RAD;
}

void MotorCtrl_SyncTrajectory(void)
{
    /* Re-seed the S-curve planner from the current Kalman position.
       Homing creep bypasses S-curve entirely, leaving s_sc stale.
       Call this before MotorCtrl_SetTarget when exiting homing mode.
       Also enables ZVD bypass — rod is not attached during homing.   */
    float pos      = s_kalman.x[0];
    s_sc.pos       = pos;
    s_sc.vel       = 0.0f;
    s_sc.acc       = 0.0f;
    s_sc.target    = pos;
    s_sc.done      = true;
    s_pos_integral = 0.0f;
    s_spd_integral = 0.0f;
    s_vel_sp       = 0.0f;
    s_acc_sp       = 0.0f;
    s_zvd_bypass   = true;
    for (uint8_t i = 0; i < ZVD_BUF_SIZE; i++) s_zvd_buf[i] = pos;
}

void MotorCtrl_Zero(float home_offset_rad)
{
    /* Redefine the current physical position as home + offset.
       All PID/Kalman state is re-seeded from the new origin.                */
    s_homing_mode  = false;
    s_pos_counts   = (int32_t)(home_offset_rad * RAD_TO_COUNTS);
    Kalman_Init(&s_kalman, home_offset_rad);
    s_sc.pos       = home_offset_rad;
    s_sc.vel       = 0.0f;
    s_sc.acc       = 0.0f;
    s_sc.target    = home_offset_rad;
    s_sc.done      = true;
    s_pos_integral = 0.0f;
    s_spd_integral = 0.0f;
    s_vel_sp       = 0.0f;
    s_acc_sp       = 0.0f;
    s_running      = false;

    /* Re-enable ZVD and pre-fill buffer — rod will be attached for normal moves */
    s_zvd_bypass = false;
    for (uint8_t i = 0; i < ZVD_BUF_SIZE; i++) s_zvd_buf[i] = home_offset_rad;
}

/* -------------------------------------------------------------------------
   Inner loop — 1 kHz, called from TIM6 ISR
   ------------------------------------------------------------------------- */
void MotorCtrl_Tick1kHz(void)
{
    /* E-stop guard */
    if (g_robot.sensors.estop) {
        s_vel_sp = 0.0f;
        s_acc_sp = 0.0f;
        s_spd_integral = 0.0f;
        Motor_SetPWM(0);
        return;
    }

    /* --- 1. Read encoder (16-bit counter, handles overflow) --------------- */
    uint16_t enc   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t  delta = (int16_t)(enc - s_last_enc);
    s_last_enc     = enc;
    s_pos_counts  += (int32_t)delta * ENCODER_DIRECTION;

    /* --- Cable-limit hard stop ------------------------------------------- */
    if (s_pos_counts >  CABLE_MAX_COUNTS || s_pos_counts < -CABLE_MAX_COUNTS) {
        s_vel_sp       = 0.0f;
        s_acc_sp       = 0.0f;
        s_spd_integral = 0.0f;
        s_running      = false;
        Motor_SetPWM(0);
        g_robot.fsm    = STATE_FAULT;
        return;
    }

    /* --- 2. Kalman filter update ----------------------------------------- */
    float pos_rad = (float)s_pos_counts * COUNTS_TO_RAD;
    Kalman_Update(&s_kalman, pos_rad);

    /* --- 3. Update g_robot.motion ---------------------------------------- */
    g_robot.motion.position_counts = s_pos_counts;
    g_robot.motion.velocity_rps    = s_kalman.x[1] / (2.0f * M_PI);
    g_robot.motion.accel_rps2      = s_kalman.x[2] / (2.0f * M_PI);

    /* --- 4. Inner velocity PID + feedforward ------------------------------ */
    if (!s_running) {
        Motor_SetPWM(0);
        return;
    }

    float vel_actual = s_kalman.x[1];
    float vel_err    = s_vel_sp - vel_actual;

    /* Conditional integration — pause when speed is already saturated       */
    float u_ff = FF_VELOCITY * s_vel_sp + FF_ACCEL * s_acc_sp;
    if (fabsf(u_ff) < (float)MOTOR_PWM_MAX - 1.0f) {
        s_spd_integral += vel_err * DT_INNER;
    }
    /* Clamp integral */
    if (s_spd_integral >  PID_SPEED_IMAX) s_spd_integral =  PID_SPEED_IMAX;
    if (s_spd_integral < -PID_SPEED_IMAX) s_spd_integral = -PID_SPEED_IMAX;

    float der = (vel_err - s_spd_prev_err) / DT_INNER;
    s_spd_prev_err = vel_err;

    float u = PID_SPEED_KP  * vel_err
            + PID_SPEED_KI  * s_spd_integral
            + PID_SPEED_KD  * der
            + u_ff;

    /* Clamp and apply */
    if (u >  (float)MOTOR_PWM_MAX) u =  (float)MOTOR_PWM_MAX;
    if (u < -(float)MOTOR_PWM_MAX) u = -(float)MOTOR_PWM_MAX;

    int16_t pwm = (int16_t)u;
    g_robot.motion.motor_pwm = pwm;
    Motor_SetPWM(pwm);
}

/* -------------------------------------------------------------------------
   Outer loop — 100 Hz, called from TIM6 ISR every 10th tick
   ------------------------------------------------------------------------- */
void MotorCtrl_Tick100Hz(void)
{
    if (!s_running) return;

    /* --- 1. Step S-curve trajectory -------------------------------------- */
    scurve_step(DT_OUTER);

    /* --- 2. ZVD input shaper on S-curve position output ------------------ */
    s_zvd_buf[s_zvd_head] = s_sc.pos;

    float shaped;
    if (s_zvd_bypass) {
        shaped = s_sc.pos;   /* no rod attached — pass S-curve output directly */
    } else {
        uint8_t idx_t2 = (s_zvd_head + ZVD_BUF_SIZE - ZVD_T2_STEPS) % ZVD_BUF_SIZE;
        uint8_t idx_t3 = (s_zvd_head + ZVD_BUF_SIZE - ZVD_T3_STEPS) % ZVD_BUF_SIZE;
        shaped = ZVD_A1 * s_zvd_buf[s_zvd_head]
               + ZVD_A2 * s_zvd_buf[idx_t2]
               + ZVD_A3 * s_zvd_buf[idx_t3];
    }
    s_zvd_head = (s_zvd_head + 1u) % ZVD_BUF_SIZE;

    /* --- 3. Settling counter (ZVD flush after S-curve completes) --------- */
    if (s_sc.done) {
        if (s_settle_ticks < 0xFFFFu) s_settle_ticks++;
    } else {
        s_settle_ticks = 0;
    }

    /* --- Homing creep override — bypasses S-curve and position PID ------- */
    if (s_homing_mode) {
        /* Ramp toward target velocity — never step-change to avoid harsh reversals */
        float ramp = HOMING_ACCEL_RADS2 * DT_OUTER;
        if (s_vel_sp < s_homing_vel)
            s_vel_sp = (s_vel_sp + ramp < s_homing_vel) ? s_vel_sp + ramp : s_homing_vel;
        else if (s_vel_sp > s_homing_vel)
            s_vel_sp = (s_vel_sp - ramp > s_homing_vel) ? s_vel_sp - ramp : s_homing_vel;
        s_acc_sp = 0.0f;
        return;
    }

    /* --- 4. Outer position PID (derivative on measurement) --------------- */
    float pos_actual = s_kalman.x[0];
    float pos_err    = shaped - pos_actual;

    s_pos_integral += pos_err * DT_OUTER;
    if (s_pos_integral >  PID_POS_IMAX) s_pos_integral =  PID_POS_IMAX;
    if (s_pos_integral < -PID_POS_IMAX) s_pos_integral = -PID_POS_IMAX;

    float d_meas = -(pos_actual - s_pos_prev_meas) * (float)OUTER_LOOP_HZ;
    s_pos_prev_meas = pos_actual;

    float vel_cmd = PID_POS_KP * pos_err
                  + PID_POS_KI * s_pos_integral
                  + PID_POS_KD * d_meas;

    /* Clamp velocity command to ±Vmax */
    if (vel_cmd >  SCURVE_VMAX_RADS) vel_cmd =  SCURVE_VMAX_RADS;
    if (vel_cmd < -SCURVE_VMAX_RADS) vel_cmd = -SCURVE_VMAX_RADS;

    /* Pass to inner loop — volatile write is atomic enough for float on M4  */
    s_vel_sp = vel_cmd;
    s_acc_sp = s_sc.acc;
}

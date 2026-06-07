#include "motor_controller.h"
#include "kalman.h"
#include "hw_io.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include "modbus_bridge.h"       /* g_at, AT_CMD_*, AT_LOOP_*, AT_STATUS_*    */
#include "uart_dma_manager.h"    /* UartDma_SendTelemetry_T()                 */
#include "test_phase2_safety.h"  /* g_test_inj injection hooks for commissioning */
#include <math.h>
#include <string.h>
#include <stdio.h>

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
static int64_t       s_pos_counts;
static uint16_t      s_last_enc;

/* Velocity setpoint passed from outer loop to inner loop */
static volatile float s_vel_sp;
static volatile float s_acc_sp;

/* Homing creep mode — bypasses S-curve and position PID */
static volatile bool  s_homing_mode;
static volatile float s_homing_vel;

/* ---- Option 1: Velocity bypass jog ----------------------------------------
   s_jog_active   : true  → Tick100Hz feeds s_jog_vel_cmd directly to inner loop,
                            skipping S-curve, ZVD, and the position PID entirely.
   s_jog_vel_cmd  : velocity setpoint (rad/s) commanded by the joystick.
                    Written from main-loop (JogVelocity), read in Tick100Hz ISR.
                    Declared volatile so the compiler does not cache it in a register
                    across the ISR boundary.                                       */
static volatile bool  s_jog_active;
static volatile float s_jog_vel_cmd;

/* ---- Option 2: Jog step mode flag -----------------------------------------
   s_jog_step  : true while jog step mode is engaged.  Guards TuningParams.zvd.bypass
                 from being overwritten by the live-expression debugger toggle
                 every outer-loop tick.
   TuningParams.scurve.amax_rads2/jmax : runtime S-curve limits; equal to SCURVE_ defaults during
                 normal moves and jog — no override needed.                    */
static bool  s_jog_step;

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

/* -------------------------------------------------------------------------
   Auto-tune relay state machine
   State lives here; g_at (modbus_bridge) holds the cross-module parameters.
   ------------------------------------------------------------------------- */
typedef enum {
    AT_SM_IDLE     = 0,
    AT_SM_SETTLING = 1,
    AT_SM_ACTIVE   = 2,
} AtSm_t;

static AtSm_t   s_at_sm;
static uint8_t  s_at_loop;        /* latched AT_LOOP_* at relay start        */
static float    s_at_sp_rad;      /* latched setpoint, radians               */
static float    s_at_amp;         /* latched amplitude (rad/s or PWM)        */
static float    s_at_hyst_rad;    /* latched hysteresis, radians             */
static int8_t   s_at_sign;        /* current relay output polarity (+1/-1)   */
static uint16_t s_at_half_cyc;    /* relay sign-flips since ACTIVE           */
static uint32_t s_at_settle_t0;   /* HAL_GetTick() when SETTLING began       */

/* Telemetry formatting buffer — written only from Tick100Hz (TIM6 ISR),
   so single-context; no re-entrancy concern.                                */

/* -------------------------------------------------------------------------
   Software safety stack — 1-ms persistence counters
   ------------------------------------------------------------------------- */
static uint16_t s_enc_stall_cnt;    /* encoder health: ticks at high PWM + zero delta  */
static uint16_t s_overcurrent_cnt;  /* current fuse:   ticks above CURRENT_FAULT_AMPS  */
static uint16_t s_tracking_err_cnt; /* tracking error: ticks with |target−pos| > 10°   */

static void safety_trip(uint8_t code)
{
    s_vel_sp       = 0.0f;
    s_acc_sp       = 0.0f;
    s_spd_integral = 0.0f;
    s_running      = false;
    Motor_SetPWM(0);
    if (RobotState.fsm != STATE_FAULT) {
        RobotState.fsm              = STATE_FAULT;
        RobotState.comms.fault_code = code;
    }
}

/* -------------------------------------------------------------------------
   S-curve helpers
   ------------------------------------------------------------------------- */

/* Conservative stopping distance from current (vel, acc) in direction of motion.
   Uses the S-curve braking profile: jerk-down phase + deceleration phase.   */
static float scurve_stop_dist(float vel, float acc)
{
    if (vel <= 0.0f) return 0.0f;

    /* Uses runtime TuningParams.scurve.jmax_rads3 / TuningParams.scurve.amax_rads2 so Option-2 jog can raise limits
       without changing the stopping-distance calculation logic.              */
    float t_a = (acc > 0.0f) ? acc / TuningParams.scurve.jmax_rads3 : 0.0f;
    float v_a = vel + acc * t_a - 0.5f * TuningParams.scurve.jmax_rads3 * t_a * t_a;
    float d_a = vel * t_a + 0.5f * acc * t_a * t_a
                - (1.0f / 6.0f) * TuningParams.scurve.jmax_rads3 * t_a * t_a * t_a;

    float d_bc = 0.0f;
    if (v_a > 0.0f) {
        d_bc = (v_a * v_a) / (2.0f * TuningParams.scurve.amax_rads2) + (v_a * TuningParams.scurve.amax_rads2) / (2.0f * TuningParams.scurve.jmax_rads3);
    }

    return d_a + d_bc;
}

static void scurve_step(float dt)
{
    if (s_sc.done) return;

    float dist = s_sc.target - s_sc.pos;

    /* Done if very close and nearly stopped. 
       Thresholds widened to prevent discrete time chattering on tiny moves. */
    if (fabsf(dist) < 0.05f && fabsf(s_sc.vel) < 0.5f) {
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

    /* Runtime limits: TuningParams.scurve.amax_rads2 / TuningParams.scurve.jmax_rads3 are the defaults from params.h
       during normal operation.  JogStepEngage() raises them temporarily for
       Option-2 discrete stepping so each step completes before the next     */
    float jerk;
    if (abs_dist <= stop_dist) {
        jerk = -TuningParams.scurve.jmax_rads3 * dir;
    } else if (vel_dir >= TuningParams.scurve.vmax_rads - 0.01f) {
        jerk = (acc_dir > 0.01f) ? -TuningParams.scurve.jmax_rads3 * dir : 0.0f;
    } else if (acc_dir < TuningParams.scurve.amax_rads2 - 0.01f) {
        jerk = TuningParams.scurve.jmax_rads3 * dir;
    } else {
        jerk = 0.0f;
    }

    s_sc.acc += jerk * dt;
    if (s_sc.acc >  TuningParams.scurve.amax_rads2) s_sc.acc =  TuningParams.scurve.amax_rads2;
    if (s_sc.acc < -TuningParams.scurve.amax_rads2) s_sc.acc = -TuningParams.scurve.amax_rads2;

    s_sc.vel += s_sc.acc * dt;
    if (s_sc.vel >  TuningParams.scurve.vmax_rads) s_sc.vel =  TuningParams.scurve.vmax_rads;
    if (s_sc.vel < -TuningParams.scurve.vmax_rads) s_sc.vel = -TuningParams.scurve.vmax_rads;

    s_sc.pos += s_sc.vel * dt;

    /* Anti-chatter: If we crossed the target this tick, snap to it.
       This completely prevents infinite discrete-time oscillation on tiny moves
       where huge jerk values cause repeated target overshooting. */
    float new_dist = s_sc.target - s_sc.pos;
    if ((dist > 0.0f && new_dist <= 0.0f) || (dist < 0.0f && new_dist >= 0.0f)) {
        s_sc.pos  = s_sc.target;
        s_sc.vel  = 0.0f;
        s_sc.acc  = 0.0f;
        s_sc.done = true;
    }
}

/* -------------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------------- */

void MotorCtrl_SetPidGains(uint8_t loop, float kp, float ki, float kd)
{
    /* Three-float update must be atomic relative to TIM6 ISR reads.
       __disable_irq disables all maskable interrupts; the window is
       3 × STR instructions ≈ 18 ns at 170 MHz — negligible jitter.         */
    __disable_irq();
    if (loop == 1) { // 1 = Position loop
        TuningParams.pos_pid.kp   = kp;
        TuningParams.pos_pid.ki   = ki;
        TuningParams.pos_pid.kd   = kd;
        s_pos_integral = 0.0f;   /* bumpless transition */
    } else {         // 0 = Velocity loop
        TuningParams.spd_pid.kp   = kp;
        TuningParams.spd_pid.ki   = ki;
        TuningParams.spd_pid.kd   = kd;
        s_spd_integral = 0.0f;
    }
    __enable_irq();
}

float MotorCtrl_GetKp(uint8_t loop)
{
    return (loop == 1) ? TuningParams.pos_pid.kp : TuningParams.spd_pid.kp;
}

float MotorCtrl_GetKi(uint8_t loop)
{
    return (loop == 1) ? TuningParams.pos_pid.ki : TuningParams.spd_pid.ki;
}

float MotorCtrl_GetKd(uint8_t loop)
{
    return (loop == 1) ? TuningParams.pos_pid.kd : TuningParams.spd_pid.kd;
}

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
    s_zvd_head         = 0;
    s_settle_ticks     = 0;
    s_running          = false;
    s_enc_stall_cnt    = 0;
    s_overcurrent_cnt  = 0;
    s_tracking_err_cnt = 0;

    /* Jog state — both options start disengaged */
    s_jog_active   = false;
    s_jog_vel_cmd  = 0.0f;

    /* Runtime S-curve limits — match hardware-identified defaults */
    TuningParams.scurve.amax_rads2 = SCURVE_AMAX_RADS2;
    TuningParams.scurve.jmax_rads3 = SCURVE_JMAX_RADS3;

    /* Auto-tune relay state */
    s_at_sm      = AT_SM_IDLE;
    s_at_sign    = 1;
    s_at_half_cyc = 0;

    /* Restore live PID gains to params.h defaults on re-init */
    TuningParams.spd_pid.kp = PID_SPEED_KP;
    TuningParams.spd_pid.ki = PID_SPEED_KI;
    TuningParams.spd_pid.kd = PID_SPEED_KD;
    TuningParams.pos_pid.kp = PID_POS_KP;
    TuningParams.pos_pid.ki = PID_POS_KI;
    TuningParams.pos_pid.kd = PID_POS_KD;

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
    s_vel_sp           = 0.0f;
    s_acc_sp           = 0.0f;
    s_spd_integral     = 0.0f;
    s_pos_integral     = 0.0f;
    s_running          = false;
    Motor_SetPWM(0);
    /* Re-seed S-curve from current position so the next SetTarget starts clean */
    float cur       = s_kalman.x[0];
    s_sc.pos        = cur;
    s_sc.vel        = 0.0f;
    s_sc.acc        = 0.0f;
    s_sc.target     = cur;
    s_sc.done       = true;
    s_enc_stall_cnt    = 0;
    s_overcurrent_cnt  = 0;
    s_tracking_err_cnt = 0;
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

void MotorCtrl_HomingCreepVel(int8_t dir, float vel_rads)
{
    s_homing_mode = true;
    s_homing_vel  = (dir >= 0) ? vel_rads : -vel_rads;
    s_running     = true;
}

void MotorCtrl_HomingCreep(int8_t dir)            /* legacy wrapper, unchanged behaviour */
{
    MotorCtrl_HomingCreepVel(dir, HOMING_VEL_RADS);
}

bool MotorCtrl_IsAtPosition(void)
{
    /* Like IsAtTarget but without the velocity gate.
       When ZVD is bypassed (homing, no rod) only a short settle is needed.
       When ZVD is active the full tail (T3+10 ticks) must flush first.    */
    uint16_t need = TuningParams.zvd.bypass ? 5u : (uint16_t)(ZVD_T3_STEPS + 10u);
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
    TuningParams.zvd.bypass   = true;
    for (uint8_t i = 0; i < ZVD_BUF_SIZE; i++) s_zvd_buf[i] = pos;
}

void MotorCtrl_Zero(float home_offset_rad)
{
    /* Redefine the current physical position as home + offset.
       All PID/Kalman state is re-seeded from the new origin.                */
    s_homing_mode  = false;
    s_pos_counts   = (int64_t)(home_offset_rad * RAD_TO_COUNTS);
    
    /* Reset the physical hardware timer counter to match the new logical position */
    uint16_t new_enc = (uint16_t)(s_pos_counts & 0xFFFF);
    __HAL_TIM_SET_COUNTER(&htim3, new_enc);
    s_last_enc     = new_enc;

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
    TuningParams.zvd.bypass = false;
    for (uint8_t i = 0; i < ZVD_BUF_SIZE; i++) s_zvd_buf[i] = home_offset_rad;
}

/* -------------------------------------------------------------------------
   Inner loop — 1 kHz, called from TIM6 ISR
   ------------------------------------------------------------------------- */
void MotorCtrl_Tick1kHz(void)
{
    /* E-stop hard guard — only after stop-and-verify has CONFIRMED the fault.
       Skipped entirely when estop_disabled = true (debug bypass).          */
    if (!RobotState.dbg.estop_disabled && RobotState.comms.fault_code == 0x01u) {
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

    /* [HOOK A] Guard 1 — suppress delta to simulate encoder cable disconnect */
    if (g_test_inj.inject_enc_stall) { delta = 0; }

    s_pos_counts  += (int64_t)delta * ENCODER_DIRECTION;

    /* [HOOK B] Guard 2 — force position past cable limit */
    if (g_test_inj.inject_boundary) {
        s_pos_counts = (int64_t)CABLE_MAX_COUNTS + 1;
    }

    /* --- Boundary guard (Guard 2) — always active, no enable flag ---------- */
    if (s_pos_counts >  CABLE_MAX_COUNTS || s_pos_counts < -CABLE_MAX_COUNTS) {
        safety_trip(0x41u);
        RobotState.dbg.safety.tripped_boundary = true;
        return;
    }

    /* --- 2. Kalman filter update ----------------------------------------- */
    float pos_rad = (float)s_pos_counts * COUNTS_TO_RAD;
    Kalman_Update(&s_kalman, pos_rad);

    /* --- 3. Update RobotState.motion ---------------------------------------- */
    RobotState.motion.position_counts = s_pos_counts;
    RobotState.motion.current_pos_deg = (float)s_pos_counts * (360.0f / (float)ENCODER_CPR);
    RobotState.motion.velocity_rps    = s_kalman.x[1] / (2.0f * M_PI);
    RobotState.motion.accel_rps2      = s_kalman.x[2] / (2.0f * M_PI);

    /* --- 4a. Auto-tune VELOCITY relay (inner loop, AT_LOOP_VELOCITY) ------- *
     * Intercepts here — after Kalman, before normal PID.  E-stop guard above *
     * already returned so no double-guard needed.                             *
     * ────────────────────────────────────────────────────────────────────── */
    if (g_at.cmd == AT_CMD_START_RELAY && g_at.loop_target == AT_LOOP_VELOCITY) {

        /* ── Abort / cleanup (relay was active, cmd now cleared) ─────────── */
        if (s_at_sm != AT_SM_IDLE && g_at.cmd != AT_CMD_START_RELAY) {
            /* unreachable here since cmd IS START_RELAY, kept for symmetry   */
        }

        /* ── Edge trigger: transition IDLE → SETTLING ───────────────────── */
        if (s_at_sm == AT_SM_IDLE) {
            s_at_loop      = AT_LOOP_VELOCITY;
            s_at_amp       = g_at.amplitude;                         /* PWM   */
            s_at_sp_rad    = g_at.setpoint;                          /* rad/s */
            s_at_hyst_rad  = g_at.hysteresis;                        /* rad/s */
            s_at_sign      = 1;
            s_at_half_cyc  = 0;
            s_at_settle_t0 = HAL_GetTick();
            s_at_sm        = AT_SM_SETTLING;
            g_at.status    = AT_STATUS_SETTLING;
            g_at.cycles    = 0;
        }

        /* ── SETTLING: hold PWM=0 until settle window elapses ───────────── */
        if (s_at_sm == AT_SM_SETTLING) {
            if ((HAL_GetTick() - s_at_settle_t0) >= AT_SETTLE_MS) {
                s_at_sm     = AT_SM_ACTIVE;
                g_at.status = AT_STATUS_ACTIVE;
            }
            Motor_SetPWM(0);
            return;
        }

        /* ── ACTIVE: bang-bang relay with hysteresis ─────────────────────── */
        if (s_at_sm == AT_SM_ACTIVE) {
            float pv  = s_kalman.x[1];                /* Kalman velocity, rad/s */
            float err = s_at_sp_rad - pv;

            if (s_at_sign > 0 && err < -(s_at_hyst_rad * 0.5f)) {
                s_at_sign = -1;
                s_at_half_cyc++;
            } else if (s_at_sign < 0 && err > (s_at_hyst_rad * 0.5f)) {
                s_at_sign = 1;
                s_at_half_cyc++;
            }
            g_at.cycles = s_at_half_cyc >> 1u;        /* complete cycles     */

            int16_t pwm = (int16_t)lroundf((float)s_at_sign * s_at_amp);
            if (pwm >  (int16_t)MOTOR_PWM_MAX) pwm =  (int16_t)MOTOR_PWM_MAX;
            if (pwm < -(int16_t)MOTOR_PWM_MAX) pwm = -(int16_t)MOTOR_PWM_MAX;
            RobotState.motion.motor_pwm = pwm;
            Motor_SetPWM(pwm);
            return;
        }
    } else if (s_at_sm != AT_SM_IDLE && s_at_loop == AT_LOOP_VELOCITY) {
        /* Relay was running on velocity loop but cmd was cleared → safe stop */
        s_at_sm        = AT_SM_IDLE;
        g_at.status    = AT_STATUS_IDLE;
        s_vel_sp       = 0.0f;
        s_acc_sp       = 0.0f;
        s_spd_integral = 0.0f;
        Motor_SetPWM(0);
    }

    /* --- 4b. Normal inner velocity PID + feedforward ---------------------- */
    if (!s_running) {
        Motor_SetPWM(0);
        return;
    }

    /* [HOOK C] Guard 4 — sustain a large Kalman position error to simulate jam.
       Placed after the !s_running gate so s_sc.done is still visible to the
       tracking guard that runs further below.                                 */
    if (g_test_inj.inject_tracking_error) {
        s_kalman.x[0] = s_sc.target + 0.30f;
    }

    float vel_actual = s_kalman.x[1];
    float vel_err    = s_vel_sp - vel_actual;

    /* Conditional integration — pause when speed is already saturated       */
    float u_ff = TuningParams.ff.velocity * s_vel_sp + TuningParams.ff.accel * s_acc_sp;
    if (!s_homing_mode) {
        if (s_vel_sp > 0.01f) {
            u_ff += TuningParams.ff.disturbance;
        } else if (s_vel_sp < -0.01f) {
            u_ff -= TuningParams.ff.disturbance;
        }
    }
    if (fabsf(u_ff) < (float)MOTOR_PWM_MAX - 1.0f) {
        s_spd_integral += vel_err * DT_INNER;
    }
    /* Clamp integral so its maximum PWM contribution is PID_SPEED_IMAX */
    float max_integral = PID_SPEED_IMAX;
    if (TuningParams.spd_pid.ki > 0.001f) {
        max_integral = PID_SPEED_IMAX / TuningParams.spd_pid.ki;
    }
    if (s_spd_integral >  max_integral) s_spd_integral =  max_integral;
    if (s_spd_integral < -max_integral) s_spd_integral = -max_integral;

    float der = (vel_err - s_spd_prev_err) / DT_INNER;
    s_spd_prev_err = vel_err;

    float u = TuningParams.spd_pid.kp * vel_err          /* live-adjustable gains       */
            + TuningParams.spd_pid.ki * s_spd_integral
            + TuningParams.spd_pid.kd * der
            + u_ff;

    /* Clamp and apply — round (not truncate) so sub-1 values reach motor */
    if (u >  (float)MOTOR_PWM_MAX) u =  (float)MOTOR_PWM_MAX;
    if (u < -(float)MOTOR_PWM_MAX) u = -(float)MOTOR_PWM_MAX;

    int16_t pwm = (int16_t)lroundf(u);

    /* Dead-zone compensation is now handled by Coulomb friction feedforward (TuningParams.ff.disturbance). */

    RobotState.motion.motor_pwm = pwm;
    Motor_SetPWM(pwm);

    /* --- Software Safety Stack ------------------------------------------- */

    /* Guard 1: Encoder health — detect broken/disconnected encoder cable.
       If the motor is being driven (|PWM| > threshold) but the encoder shows
       no movement for SAFETY_ENC_STALL_MS consecutive ticks → fault 0x40.   */
    if (RobotState.dbg.safety.en_encoder_health) {
        if ((pwm > SAFETY_ENC_STALL_PWM || pwm < -SAFETY_ENC_STALL_PWM) && delta == 0) {
            if (++s_enc_stall_cnt >= SAFETY_ENC_STALL_MS) {
                safety_trip(0x40u);
                RobotState.dbg.safety.tripped_encoder = true;
                s_enc_stall_cnt = 0;
            }
        } else {
            s_enc_stall_cnt = 0;
        }
    } else {
        s_enc_stall_cnt = 0;
    }

    /* Guard 3: Persistent current fuse — overcurrent jam / stall detection.
       100 ms persistence filters WCS1800 sensor noise and motion transients. */
    if (RobotState.dbg.safety.en_current_safety) {
        if (RobotState.sensors.current_amps > CURRENT_FAULT_AMPS) {
            if (++s_overcurrent_cnt >= SAFETY_CURRENT_MS) {
                safety_trip(0x42u);
                RobotState.dbg.safety.tripped_current = true;
                s_overcurrent_cnt = 0;
            }
        } else {
            s_overcurrent_cnt = 0;
        }
    } else {
        s_overcurrent_cnt = 0;
    }

    /* Guard 4: Tracking error — mechanical jam after move completes.
       Gate on s_sc.done: during cruise the cascade lag can exceed 100°, so
       checking mid-move would false-trip.  After the S-curve finishes the arm
       must settle within SAFETY_TRACKING_DEG in SAFETY_TRACKING_MS ticks.   */
    if (RobotState.dbg.safety.en_tracking_safety && !s_homing_mode && s_sc.done) {
        float err_rad = fabsf(s_sc.target - s_kalman.x[0]);
        if (err_rad > SAFETY_TRACKING_DEG * (M_PI / 180.0f)) {
            if (++s_tracking_err_cnt >= SAFETY_TRACKING_MS) {
                safety_trip(0x43u);
                RobotState.dbg.safety.tripped_tracking = true;
                s_tracking_err_cnt = 0;
            }
        } else {
            s_tracking_err_cnt = 0;
        }
    } else {
        s_tracking_err_cnt = 0;
    }
}

/* -------------------------------------------------------------------------
   Outer loop — 100 Hz, called from TIM6 ISR every 10th tick
   ------------------------------------------------------------------------- */
void MotorCtrl_Tick100Hz(void)
{
    /* =========================================================
       Auto-tune POSITION relay (outer loop, AT_LOOP_POSITION)

       Placed BEFORE the s_running gate so the relay can run
       even when no normal trajectory is active.

       ISR time budget: relay logic ~200 ns, telemetry snprintf
       ~4 µs — total < 0.05 % of the 10 ms outer-loop window.
       ========================================================= */

    if (g_at.cmd == AT_CMD_START_RELAY && g_at.loop_target == AT_LOOP_POSITION) {

        /* ── Edge trigger: IDLE → SETTLING ──────────────────────────── */
        if (s_at_sm == AT_SM_IDLE) {
            s_at_loop     = AT_LOOP_POSITION;
            s_at_amp      = g_at.amplitude;                     /* rad/s */
            s_at_sp_rad   = g_at.setpoint * (M_PI / 180.0f);   /* rad   */
            s_at_hyst_rad = g_at.hysteresis * (M_PI / 180.0f); /* rad   */
            s_at_sign     = 1;
            s_at_half_cyc = 0;
            s_at_settle_t0 = HAL_GetTick();
            s_at_sm       = AT_SM_SETTLING;
            g_at.status   = AT_STATUS_SETTLING;
            g_at.cycles   = 0;
        }

        /* ── SETTLING: hold velocity command to 0 ────────────────────── */
        if (s_at_sm == AT_SM_SETTLING) {
            if ((HAL_GetTick() - s_at_settle_t0) >= AT_SETTLE_MS) {
                s_at_sm     = AT_SM_ACTIVE;
                g_at.status = AT_STATUS_ACTIVE;
            }
            s_vel_sp = 0.0f;
            s_acc_sp = 0.0f;
            goto send_telemetry;   /* still emit telemetry during settle    */
        }

        /* ── ACTIVE: bang-bang relay with hysteresis ─────────────────── */
        if (s_at_sm == AT_SM_ACTIVE) {
            float pv  = s_kalman.x[0];              /* Kalman position, rad */
            float err = s_at_sp_rad - pv;

            /* Schmitt-trigger relay switching */
            if (s_at_sign > 0 && err < -(s_at_hyst_rad * 0.5f)) {
                s_at_sign = -1;
                s_at_half_cyc++;
            } else if (s_at_sign < 0 && err > (s_at_hyst_rad * 0.5f)) {
                s_at_sign = 1;
                s_at_half_cyc++;
            }
            g_at.cycles = s_at_half_cyc >> 1u;     /* complete cycles       */

            /* Directly command velocity setpoint — inner PID does the rest */
            s_vel_sp = (float)s_at_sign * s_at_amp;
            s_acc_sp = 0.0f;
            goto send_telemetry;
        }

    } else if (s_at_sm != AT_SM_IDLE && s_at_loop == AT_LOOP_POSITION) {
        /* cmd cleared (Abort or Idle) while position relay was running      */
        s_at_sm        = AT_SM_IDLE;
        g_at.status    = AT_STATUS_IDLE;
        s_vel_sp       = 0.0f;
        s_acc_sp       = 0.0f;
        s_spd_integral = 0.0f;
        s_pos_integral = 0.0f;
        Motor_SetPWM(0);
        /* Fall through to normal path — s_running will gate correctly      */
    }

    /* TuningParams.zvd.bypass is used directly; no override needed.
       JogStepEngage sets it directly.   */

    if (!s_running) goto send_telemetry;

    /* ====================================================================
       OPTION 1 — Velocity Bypass Jog
       When s_jog_active is true the entire outer loop (S-curve trajectory,
       ZVD shaper, position PID) is bypassed.  The joystick velocity command
       is written directly to s_vel_sp so the inner 1 kHz velocity PID
       tracks it without position-loop interference.

       Integral housekeeping (critical for bump-less re-engagement):
         s_pos_integral  : zeroed every outer tick.  If we let it accumulate
                           while the motor drifts under velocity control the
                           position PID would fire a large correction burst
                           the moment JogRelease() re-engages the outer loop.
         s_pos_prev_meas : updated to the current Kalman position every tick.
                           This pre-seeds the derivative-on-measurement term
                           so d/dt(pos_actual) = 0 on the first post-jog tick
                           rather than reflecting the full position change
                           since the last time the outer loop was active
                           (derivative kick prevention).
       ==================================================================== */
    if (s_jog_active) {
        s_pos_integral  = 0.0f;
        s_pos_prev_meas = s_kalman.x[0];
        /* Ramp toward commanded velocity — same approach as homing creep.
           Prevents harsh direction reversals when the arm is still coasting
           from a previous move, which can stall the motor at low PWM.        */
        {
            float ramp = JOG_PC_ACCEL_RADS2 * DT_OUTER;
            if (s_vel_sp < s_jog_vel_cmd)
                s_vel_sp = (s_vel_sp + ramp > s_jog_vel_cmd) ? s_jog_vel_cmd : s_vel_sp + ramp;
            else
                s_vel_sp = (s_vel_sp - ramp < s_jog_vel_cmd) ? s_jog_vel_cmd : s_vel_sp - ramp;
        }
        s_acc_sp = 0.0f;
        goto send_telemetry;
    }

    /* --- 1. Step S-curve trajectory -------------------------------------- */
    scurve_step(DT_OUTER);

    /* --- 2. ZVD input shaper on S-curve position output ------------------ */
    s_zvd_buf[s_zvd_head] = s_sc.pos;

    float shaped;
    if (TuningParams.zvd.bypass) {
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
        goto send_telemetry;
    }

    /* --- 4. Outer position PID (derivative on measurement) --------------- */
    float pos_actual = s_kalman.x[0];
    float pos_err    = shaped - pos_actual;

    /* Position deadband: if error is within ±POS_DEADBAND_RAD, treat as zero.
       Without this the PID produces a sub-1-count PWM that lroundf truncates
       to 0, the integral slowly winds, then kicks 1 PWM count which overshoots,
       causing 1° stick-slip hunting indefinitely around the target.
       Only apply deadband when S-curve is done and ZVD is flushed. */
    bool zvd_flushed = TuningParams.zvd.bypass || (s_settle_ticks >= ZVD_T3_STEPS);
    if (s_sc.done && zvd_flushed && fabsf(pos_err) < POSITION_DEADBAND_RAD) {
        s_vel_sp = 0.0f;
        s_acc_sp = 0.0f;
        /* Do not integrate inside the deadband — prevents windup while parked */
        s_pos_prev_meas = pos_actual;
        goto send_telemetry;
    }

    /* Anti-stiction boost: if S-Curve is done but we are stuck outside the deadband,
       the normal Ki=0.02 takes 10s to build up enough PWM to break static friction.
       Boost Ki dynamically when velocity is near zero to settle quickly. */
    float dynamic_ki = TuningParams.pos_pid.ki;
    if (s_sc.done && fabsf(s_kalman.x[1]) < VELOCITY_SETTLED_RADS) {
        dynamic_ki *= 25.0f; /* Winds up in ~400ms instead of 10s */
    }

    if (s_sc.done) {
        s_pos_integral += pos_err * DT_OUTER;
        if (s_pos_integral >  PID_POS_IMAX) s_pos_integral =  PID_POS_IMAX;
        if (s_pos_integral < -PID_POS_IMAX) s_pos_integral = -PID_POS_IMAX;
    } else {
        s_pos_integral = 0.0f;
    }

    float d_meas = -(pos_actual - s_pos_prev_meas) * (float)OUTER_LOOP_HZ;
    s_pos_prev_meas = pos_actual;

    float vel_cmd = TuningParams.pos_pid.kp * pos_err          /* live-adjustable gains */
                  + dynamic_ki * s_pos_integral
                  + TuningParams.pos_pid.kd * d_meas;

    /* Clamp velocity command to ±Vmax */
    if (vel_cmd >  TuningParams.scurve.vmax_rads) vel_cmd =  TuningParams.scurve.vmax_rads;
    if (vel_cmd < -TuningParams.scurve.vmax_rads) vel_cmd = -TuningParams.scurve.vmax_rads;

    /* Pass to inner loop — volatile write is atomic enough for float on M4  */
    s_vel_sp = vel_cmd;
    s_acc_sp = s_sc.acc;

send_telemetry:
    /* ── Telemetry at 100 Hz ────────────────────────────────────────────────
       Format: $T,{ms},{pos_deg×10},{vel_rads×10},{acc_rads2×10},{pwm×10}\r\n
       Integer-only arithmetic — no float formatting, no malloc.
       snprintf + ring-buffer write: ~4 µs @ 170 MHz = 0.04 % of 10 ms slot.
       UartDma_SendTelemetry_T() drops silently if the TX watermark is hit.
       ─────────────────────────────────────────────────────────────────────── */
    {
        /* 1. Send standard $T telemetry for compatibility with existing dashboard */
        int16_t pos_x10 = (int16_t)lroundf(s_kalman.x[0] * (180.0f / M_PI) * 10.0f);
        int16_t vel_x10 = (int16_t)lroundf(s_kalman.x[1] * 10.0f);
        int16_t acc_x10 = (int16_t)lroundf(s_kalman.x[2] * 10.0f);
        int16_t co_x10  = (int16_t)(RobotState.motion.motor_pwm * 10);
        int16_t vel_set_x10 = (int16_t)lroundf(s_vel_sp * 10.0f);
        if (!UartDma_SendTelemetry_T(HAL_GetTick(), pos_x10, vel_x10, acc_x10, co_x10, vel_set_x10)) {
            RobotState.comms.telemetry_drops++;
        }

        /* 2. Send detailed $CTRL debug telemetry for the tuning dashboard */
        static char ctrl_buf[140];
        int16_t sc_tgt_x10  = (int16_t)lroundf(s_sc.target * (180.0f / M_PI) * 10.0f);
        int16_t sc_pos_x10  = (int16_t)lroundf(s_sc.pos * (180.0f / M_PI) * 10.0f);
        int16_t sh_pos_x10  = (int16_t)lroundf(shaped * (180.0f / M_PI) * 10.0f);
        int16_t kal_pos_x100 = (int16_t)lroundf(s_kalman.x[0] * (180.0f / M_PI) * 100.0f);
        int16_t pos_err_x100 = (int16_t)lroundf(pos_err * (180.0f / M_PI) * 100.0f);
        int16_t vel_sp_x10  = (int16_t)lroundf(s_vel_sp * (180.0f / M_PI) * 10.0f);
        int16_t kal_vel_x10 = (int16_t)lroundf(s_kalman.x[1] * (180.0f / M_PI) * 10.0f);
        int16_t vel_err_x10 = (int16_t)lroundf((s_vel_sp - s_kalman.x[1]) * (180.0f / M_PI) * 10.0f);
        int16_t pwm_x10     = (int16_t)(RobotState.motion.motor_pwm * 10);
        int16_t pos_int_x10 = (int16_t)lroundf(s_pos_integral * (180.0f / M_PI) * 10.0f);
        int16_t spd_int_x10 = (int16_t)lroundf(s_spd_integral * 10.0f);

        snprintf(ctrl_buf, sizeof(ctrl_buf),
                 "$CTRL,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 (unsigned long)HAL_GetTick(),
                 sc_tgt_x10, sc_pos_x10, sh_pos_x10, kal_pos_x100, pos_err_x100,
                 vel_sp_x10, kal_vel_x10, vel_err_x10, pwm_x10, pos_int_x10, spd_int_x10);

        UartDma_SendTelemetry(ctrl_buf);
    }
}

void MotorCtrl_SetZvdBypass(bool bypass)
{
    TuningParams.zvd.bypass = bypass;
}

/* ==========================================================================
   OPTION 1 — Velocity Bypass API
   ========================================================================== */

/* MotorCtrl_JogVelocity — call from App_Run every iteration while the L/R
   joystick key is held.

   Control theory notes:
   - Bypasses the outer position loop entirely (no ZVD delay, no S-curve lag).
   - The inner velocity PID tracks vel_rads with its full 1 kHz bandwidth,
     giving the operator direct, low-latency velocity feel.
   - Safe to call from the main loop while Tick100Hz is in the ISR: s_jog_active
     and s_jog_vel_cmd are volatile, and both are simple scalar writes (atomic on
     Cortex-M4 for float in a single FPU register store).
   - s_running is set on first entry so the inner loop does not gate off.       */
void MotorCtrl_JogVelocity(float vel_rads)
{
    /* Clamp to Vmax so the operator cannot command beyond the S-curve ceiling */
    if (vel_rads >  TuningParams.scurve.vmax_rads) { vel_rads =  TuningParams.scurve.vmax_rads; }
    if (vel_rads < -TuningParams.scurve.vmax_rads) { vel_rads = -TuningParams.scurve.vmax_rads; }

    s_running = true;   /* always: re-enable if Stop() was called mid-jog */
    if (!s_jog_active) {
        /* First-entry housekeeping (main-loop context, ISR not yet aware).
           - Clear position integral NOW so the bypass block in Tick100Hz
             does not inherit any previously accumulated value.
           - Do NOT clear s_spd_integral: the inner loop was already tracking
             velocity correctly; preserving its integral gives a bumpless
             entry into velocity-only control (no speed transient at engage). */
        s_pos_integral  = 0.0f;
        s_pos_prev_meas = s_kalman.x[0];
        s_jog_active    = true;           /* last: ISR sees consistent state */
    }
    s_jog_vel_cmd = vel_rads;
}

/* MotorCtrl_JogRelease — call from App_Run when the L/R key is released.

   Bump-less re-engagement of the position loop:
   1. Zero the velocity command so the inner PID ramps down naturally.
   2. Clear s_spd_integral — it was tracking the jog velocity setpoint (e.g.
      1 rad/s), not zero.  Leaving it would inject a velocity burst on the
      first position-PID-controlled tick.
   3. Re-seed the S-curve from the current Kalman state.  Crucially, s_sc.vel
      is set to the estimated actual velocity rather than 0.  This gives the
      S-curve realistic initial conditions so it naturally decelerates the
      coasting motor to the locked target — no step change in the trajectory.
   4. Flush the ZVD buffer to the lock-on position.  Stale taps from the last
      jog move would otherwise create a one-cycle position transient.
   5. s_pos_integral is already 0 (zeroed each outer tick during jog).
      s_pos_prev_meas is already current (updated each outer tick during jog).
      Both derivative and integral terms of the position PID therefore start
      from a clean, consistent state — pure bump-less handover.               */
void MotorCtrl_JogRelease(void)
{
    if (!s_jog_active) { return; }

    /* Step 1 & 2 — velocity teardown */
    s_jog_vel_cmd  = 0.0f;
    s_vel_sp       = 0.0f;
    s_acc_sp       = 0.0f;
    s_spd_integral = 0.0f;   /* was tracking jog vel, not zero — must clear */
    s_spd_prev_err = 0.0f;

    /* Step 3 — re-seed S-curve with real plant state (bump-less trajectory) */
    {
        float pos    = s_kalman.x[0];
        float vel    = s_kalman.x[1]; /* inherit actual velocity — no step change */
        s_sc.pos     = pos;
        s_sc.vel     = vel;
        s_sc.acc     = 0.0f;
        s_sc.target  = pos;   /* lock: decelerate to current position */
        s_sc.done    = false; /* S-curve must generate the decel profile */
        s_settle_ticks = 0u;
    }

    /* Step 4 — flush ZVD buffer so all delayed taps agree on the lock position */
    {
        float pos = s_kalman.x[0];
        uint8_t i;
        for (i = 0u; i < ZVD_BUF_SIZE; i++) { s_zvd_buf[i] = pos; }
    }

    /* Step 5 — position PID state already clean (maintained during jog) */

    /* Last: clear jog flag so Tick100Hz re-engages position loop next tick */
    s_jog_active = false;
}

bool MotorCtrl_IsJogActive(void)
{
    return s_jog_active;
}

/* ==========================================================================
   OPTION 2 — Aggressive Step-Mode API
   ========================================================================== */

/* MotorCtrl_JogStepEngage — call once when the first L/R key press is detected.

   Switches Tick100Hz to the trapezoidal planner (trap_step) and bypasses ZVD.
   ZVD latency (80 ms) makes discrete jog steps feel sluggish; with no rod
   resonance to suppress during manual jogging, bypassing it is acceptable.    */
void MotorCtrl_JogStepEngage(void)
{
    /* Flush the ZVD buffer to current position — prevents stale taps from
       the last normal move from corrupting the first jog step.               */
    {
        float pos = s_kalman.x[0];
        uint8_t i;
        for (i = 0u; i < ZVD_BUF_SIZE; i++) { s_zvd_buf[i] = pos; }
    }

    s_jog_step   = true;   /* guards TuningParams.zvd.bypass from live-toggle override   */
    TuningParams.zvd.bypass = true;

    /* Seed S-curve from current state so the first SetTarget() starts clean  */
    {
        float pos    = s_kalman.x[0];
        s_sc.pos     = pos;
        s_sc.vel     = 0.0f;
        s_sc.acc     = 0.0f;
        s_sc.target  = pos;
        s_sc.done    = true;
    }
}

/* MotorCtrl_JogStepDisengage — call when the L/R key is released.

   Any small residual position error after each jog step is integrated by
   s_pos_integral.  Clearing it on disengage ensures the first post-jog
   position command starts from a clean baseline.                              */
void MotorCtrl_JogStepDisengage(void)
{
    s_jog_step = false;

    /* Re-enable ZVD.  Flush buffer so the first shaped output is current
       position — ZVD delay ramps in naturally over the next 80 ms.           */
    {
        float pos = s_kalman.x[0];
        uint8_t i;
        for (i = 0u; i < ZVD_BUF_SIZE; i++) { s_zvd_buf[i] = pos; }
    }
    TuningParams.zvd.bypass = false;

    s_pos_integral = 0.0f;
    s_spd_integral = 0.0f;
}

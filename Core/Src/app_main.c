#include "app_main.h"
#include "modbus_bridge.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
#include "joystick.h"
#include "canbus.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim6;

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* =========================================================================
   DEBUG OVERRIDE
   Set to 1 to temporarily disable all autonomous faults (Heartbeat, Joystick 
   watchdog, Homing timeouts, E-stop). Useful for uninterrupted tuning!
   ========================================================================= */
#define DISABLE_ALL_FAULTS 1

/* =========================================================================
   Homing sub-state machine
   Motion law: ALWAYS HomingCreep (HOMING_VEL_RADS). Never S-curve during home.

   Two-stage industrial homing. Motion law: ALWAYS HomingCreep. Never S-curve.

   Sequence:
     FAST_SEARCH   — growing sweep at FAST vel; coarse sensor find (edge discarded)
     BACKOFF       — reverse until prox OFF + HOMING_BACKOFF_DEG margin (clean OFF state)
     PREC_EDGE_A   — slow steady approach into sensor; capture edge A
     PREC_OVERSHOOT— continue HOMING_OVERSHOOT_DEG past edge A to clear the sensor
     FIND_EDGE_B   — reverse at precision vel; rising edge from other side = edge B
     GO_CENTER     — creep to (A + B) / 2; timeout after HOMING_CENTER_TIMEOUT_MS
     SETTLE        — hold still for HOMING_SETTLE_MS then zero

   Edges A and B are both captured at HOMING_PREC_VEL_RADS, so sensor-latency
   bias is symmetric and cancels in the (A+B)/2 midpoint → high repeatability.
   ========================================================================= */
typedef enum {
    HOM_IDLE = 0,
    HOM_INIT,
    HOM_FAST_SEARCH,    /* growing sweep at FAST vel — coarse find, edge NOT used */
    HOM_BACKOFF,        /* reverse until prox OFF + HOMING_BACKOFF_DEG margin      */
    HOM_PREC_EDGE_A,    /* slow steady approach — capture edge A                   */
    HOM_PREC_OVERSHOOT, /* continue past edge A to clear the sensor                */
    HOM_FIND_EDGE_B,    /* reversed at precision vel — rising edge = edge B        */
    HOM_GO_CENTER,      /* creep to (A + B) / 2                                    */
    HOM_SETTLE,         /* wait HOMING_SETTLE_MS then zero                         */
} HomingState_t;

static HomingState_t s_hom;
static float    s_sweep_start_rad;  /* position at start of current half-cycle */
static int8_t   s_sweep_dir;        /* +1 = forward, -1 = reverse              */
static float    s_sweep_amp_rad;    /* current half-cycle amplitude             */
static float    s_edge_a;           /* rising edge entering sensor from sweep direction       */
static float    s_edge_b;           /* rising edge entering sensor from opposite direction    */
static float    s_search_start_rad; /* position when FIND_EDGE_B begins (for safety limit)   */
static float    s_backoff_start_rad; /* position when prox first cleared during HOM_BACKOFF   */
static uint32_t s_settle_t;
static uint32_t s_center_start_ms; /* HAL_GetTick when GO_CENTER begins — for timeout        */
static uint32_t s_hom_start_ms;    /* HAL_GetTick when homing starts — for overall timeout   */
static float    s_home_offset_rad;
static float    s_user_home_rad;    /* park position saved by SetHome (post-calibration frame) */
static int8_t   s_go_center_dir;    /* tracks last dir in HOM_GO_CENTER — avoids repeated HomingCreep calls */
static uint32_t s_move_start_ms;   /* HAL_GetTick() when STATE_RUNNING was entered — move timeout ref */
static uint32_t s_tel_tick;        /* last $ST telemetry send time */
#ifndef HB_YA
#define HB_YA   22881u
#endif
#ifndef HB_HI
#define HB_HI   18537u
#endif
static uint32_t s_hb_last_tick;    /* HAL_GetTick() when reg 0x00 last changed */

/* =========================================================================
   Gripper sequence state machine
   Sequence: Down → action (Close/Open) → Up, with reed confirmation + timeout
   ========================================================================= */
typedef enum {
    GRIP_IDLE = 0,
    GRIP_PICK_DOWN,    /* going down before grab  */
    GRIP_PICK_CLOSE,   /* closing claw             */
    GRIP_PICK_UP,      /* lifting with object      */
    GRIP_PLACE_DOWN,   /* going down to release    */
    GRIP_PLACE_OPEN,   /* opening claw             */
    GRIP_PLACE_UP,     /* lifting empty            */
} GripperState_t;

static GripperState_t s_grip;
static uint32_t       s_grip_t;
static bool           s_gripper_triggered; /* auto_run: gripper started for current arrival */

/* =========================================================================
   Running-mode sub-state
   ========================================================================= */
typedef enum {
    RUN_IDLE = 0,
    RUN_JOG,
    RUN_AUTO,
    RUN_POINT,
    RUN_TEST,
} RunMode_t;

static RunMode_t s_run_mode;
static bool      s_joy_mode = false; /* true = joystick active, false = base system */

/* Jog mode tracking — only one option is active at a time.
   s_jog_vel_active  : Option 1 — velocity bypass is running.
   s_jog_step_active : Option 2 — aggressive step mode is engaged.
   s_jog_target_rad  : PC discrete step target (used by RUN_JOG velocity loop) */
static bool  s_jog_vel_active  = false;
static bool  s_jog_step_active = false;
static float s_jog_target_rad  = 0.0f;

/* Auto sequence state */
static uint8_t  s_seq_pairs;            /* number of pick+place pairs (max 8)    */
static int16_t  s_seq_slots[16];

/* Test Mode state */
static uint16_t s_test_type;
static uint16_t s_test_repeats_target;
static uint16_t s_test_repeats_done;
static int16_t  s_test_pos_a;
static int16_t  s_test_pos_b;
static bool     s_test_moving_to_b;
static uint32_t s_test_dwell_start;        /* registers 0x12–0x21 = 16 slots = 8 pairs */
static uint8_t  s_seq_step;    /* current step index (0 = pick of pair 0)   */
static bool     s_gripper_en;  /* gripper enabled in auto (reg 0x04)        */

/* =========================================================================
   Helpers
   ========================================================================= */

static float deg_to_rad(float d) { return d * (M_PI / 180.0f); }
static float rad_to_deg(float r) { return r * (180.0f / M_PI); }

static char s_dbg[80];
/* newlib-nano disables %f — values encoded as integer×100, Python decodes.
   Format: $HOM,TAG,v1x100,v2x100,POS,posx100                             */
static void hom_dbg(const char *tag, float val1, float val2, float pos_deg)
{
    snprintf(s_dbg, sizeof(s_dbg), "$HOM,%s,%ld,%ld,POS,%ld\r\n",
             tag,
             (long)lroundf(val1 * 100.0f),
             (long)lroundf(val2 * 100.0f),
             (long)lroundf(pos_deg * 100.0f));
    UartDma_SendTelemetry(s_dbg);
}

static float resolve_target(int16_t raw, uint16_t unit)
{
    float current_deg = rad_to_deg(MotorCtrl_GetPosition_rad());
    float target_deg_modulo;

    if (unit == 0u) {
        target_deg_modulo = (float)abs(raw);
    } else {
        int16_t idx = abs(raw);
        if (idx < 1 || idx > (int16_t)P2P_INDEX_COUNT) return 0.0f;
        target_deg_modulo = (float)idx * 5.0f;
    }

    /* Map target strictly to 0..359.99 */
    target_deg_modulo = fmodf(target_deg_modulo, 360.0f);
    if (target_deg_modulo < 0.0f) target_deg_modulo += 360.0f;

    /* Base angle of the current revolution */
    float base_360 = floorf(current_deg / 360.0f) * 360.0f;
    float candidate_deg = base_360 + target_deg_modulo;

    float diff = candidate_deg - current_deg;

    /* Apply direction constraint */
    if (raw > 0) {
        /* CCW: MUST move in positive direction */
        if (diff < 0.0f) candidate_deg += 360.0f;
    } else if (raw < 0) {
        /* CW: MUST move in negative direction */
        if (diff > 0.0f) candidate_deg -= 360.0f;
    }

    return deg_to_rad(candidate_deg);
}

/* Update task register 0x27                                                 */
static void set_task(uint16_t task_bits)
{
    ModbusBridge_SetReg(0x27, task_bits);
}

/* =========================================================================
   Gripper helpers
   ========================================================================= */
static void gripper_start(bool pick)
{
    Gripper_SetVertical(false); /* always go down first */
    s_grip_t = HAL_GetTick();
    s_grip   = pick ? GRIP_PICK_DOWN : GRIP_PLACE_DOWN;
}

static void gripper_seq_run(void)
{
    uint32_t now     = HAL_GetTick();
    bool     timeout = (now - s_grip_t) >= GRIP_TIMEOUT_MS;

    switch (s_grip) {
    case GRIP_IDLE: break;

    case GRIP_PICK_DOWN:
        if (HwIo_GetReedSwitch(REED_DOWN) || timeout) {
            Gripper_SetClaw(false);    /* close — grab object */
            s_grip_t = now;
            s_grip   = GRIP_PICK_CLOSE;
        }
        break;

    case GRIP_PICK_CLOSE:
        if (HwIo_GetReedSwitch(REED_CLOSE) || timeout) {
            Gripper_SetVertical(true); /* lift with object */
            s_grip_t = now;
            s_grip   = GRIP_PICK_UP;
        }
        break;

    case GRIP_PICK_UP:
        if (HwIo_GetReedSwitch(REED_UP) || timeout) {
            s_grip = GRIP_IDLE;
            ModbusBridge_SetReg(0x03, 0); /* sequence complete */
        }
        break;

    case GRIP_PLACE_DOWN:
        if (HwIo_GetReedSwitch(REED_DOWN) || timeout) {
            Gripper_SetClaw(true);     /* open — release object */
            s_grip_t = now;
            s_grip   = GRIP_PLACE_OPEN;
        }
        break;

    case GRIP_PLACE_OPEN:
        if (HwIo_GetReedSwitch(REED_OPEN) || timeout) {
            Gripper_SetVertical(true); /* lift empty */
            s_grip_t = now;
            s_grip   = GRIP_PLACE_UP;
        }
        break;

    case GRIP_PLACE_UP:
        if (HwIo_GetReedSwitch(REED_UP) || timeout) {
            s_grip = GRIP_IDLE;
            ModbusBridge_SetReg(0x03, 0); /* sequence complete */
        }
        break;
    }
}

/* =========================================================================
   Homing state machine — called every App_Run iteration
   ========================================================================= */
static void homing_run(void)
{
    float    pos  = MotorCtrl_GetPosition_rad();
    uint32_t now  = HAL_GetTick();

    /* Overall homing watchdog (Task 3): prevent endless grinding if a reed switch fails */
#if !DISABLE_ALL_FAULTS
    if (s_hom != HOM_INIT && s_hom != HOM_IDLE && (now - s_hom_start_ms) > 15000u) {
        RobotState.fsm              = STATE_FAULT;
        RobotState.comms.fault_code = 5u; /* Homing overall timeout */
        MotorCtrl_Stop();
        s_hom = HOM_IDLE;
        return;
    }
#endif

    switch (s_hom) {

    /* ------------------------------------------------------------------ */
    case HOM_INIT:
        s_hom_start_ms    = HAL_GetTick();
        s_sweep_start_rad = pos;
        s_sweep_dir       = +1;
        s_sweep_amp_rad   = deg_to_rad(HOMING_WIGGLE_STEP_DEG); /* first step */
        s_edge_a          = 0.0f;
        s_edge_b          = 0.0f;
        (void)HwIo_GetProxRisingEdge();   /* flush any stale latch from before homing */

        /* Stage 1: coarse find at FAST velocity. Whether inside or outside
           the zone, always start the sweep. This edge is discarded — it only
           locates the sensor window for the precision pass.                */
        MotorCtrl_HomingCreepVel(s_sweep_dir, HOMING_FAST_VEL_RADS);
        s_hom = HOM_FAST_SEARCH;
        break;

    /* ------------------------------------------------------------------ */
    case HOM_FAST_SEARCH:
        /* Stage 1 — triangular sweep at FAST vel until the sensor is found.
           Each time the arm travels one amplitude, reverse and grow it.    */
        {
            float traveled = (pos - s_sweep_start_rad) * (float)s_sweep_dir;

            if (traveled >= s_sweep_amp_rad) {
                /* End of half-cycle — reverse and grow */
                s_sweep_dir       = -s_sweep_dir;
                s_sweep_start_rad = pos;
                s_sweep_amp_rad  += deg_to_rad(HOMING_WIGGLE_STEP_DEG);

                if (s_sweep_amp_rad > deg_to_rad(HOMING_WIGGLE_MAX_DEG)) {
#if !DISABLE_ALL_FAULTS
                    RobotState.fsm              = STATE_FAULT;
                    RobotState.comms.fault_code = 4u; /* sensor not found in sweep */
                    MotorCtrl_Stop();
                    break;
#endif
                }
                MotorCtrl_HomingCreepVel(s_sweep_dir, HOMING_FAST_VEL_RADS);
            }

            /* Coarse hit — DO NOT record this edge. Reverse and back off at
               precision vel until the sensor clears with margin.           */
            if (HwIo_GetProxRisingEdge()) {
                hom_dbg("FST", rad_to_deg(pos), 0.0f, rad_to_deg(pos));
                s_sweep_dir         = -s_sweep_dir;
                s_sweep_start_rad   = pos;   /* backoff-distance safety ref  */
                s_backoff_start_rad = pos;   /* armed once prox clears       */
                MotorCtrl_HomingCreepVel(s_sweep_dir, HOMING_PREC_VEL_RADS);
                (void)HwIo_GetProxRisingEdge();   /* flush latch */
                s_hom = HOM_BACKOFF;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_BACKOFF:
        /* Stage 2 — drive away until prox OFF, then an extra margin so the
           precision pass always starts from a guaranteed clean OFF state.  */
        if (HwIo_GetProximity()) {
            s_backoff_start_rad = pos;        /* still inside — keep arming  */
            if (fabsf(pos - s_sweep_start_rad) > deg_to_rad(HOMING_BACKOFF_MAX_DEG)) {
#if !DISABLE_ALL_FAULTS
                RobotState.fsm              = STATE_FAULT;
                RobotState.comms.fault_code = 3u; /* sensor never cleared       */
                MotorCtrl_Stop();
#endif
            }
        } else if (fabsf(pos - s_backoff_start_rad) >= deg_to_rad(HOMING_BACKOFF_DEG)) {
            /* Fully clear + margin — reverse into sensor at PRECISION vel.  */
            s_sweep_dir        = -s_sweep_dir; /* back toward sensor          */
            s_search_start_rad = pos;
            MotorCtrl_HomingCreepVel(s_sweep_dir, HOMING_PREC_VEL_RADS);
            (void)HwIo_GetProxRisingEdge();    /* flush before edge A         */
            hom_dbg("BAK", rad_to_deg(pos), 0.0f, rad_to_deg(pos));
            s_hom = HOM_PREC_EDGE_A;
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_PREC_EDGE_A:
        /* Stage 3a — steady precision velocity → constant, known latency.  */
        if (HwIo_GetProxRisingEdge()) {
            s_edge_a = pos;
            hom_dbg("EA1", rad_to_deg(pos), rad_to_deg(s_sweep_amp_rad), rad_to_deg(pos));
            s_hom = HOM_PREC_OVERSHOOT;
        } else if (fabsf(pos - s_search_start_rad) > deg_to_rad(HOMING_MAX_SEARCH_DEG)) {
#if !DISABLE_ALL_FAULTS
            RobotState.fsm              = STATE_FAULT;
            RobotState.comms.fault_code = 2u;     /* edge A not found            */
            MotorCtrl_Stop();
#endif
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_PREC_OVERSHOOT:
        /* Stage 3b — continue past edge A until the sensor physically clears
           (prox OFF). A fixed deg stop is not enough when the sensor window
           is wider than HOMING_OVERSHOOT_DEG — the motor would reverse while
           still inside the zone, exit from the wrong side, and never find B. */
        {
            float past = (pos - s_edge_a) * (float)s_sweep_dir;
            if (past >= deg_to_rad(HOMING_OVERSHOOT_DEG) && !HwIo_GetProximity()) {
                /* Sensor cleared — motor is now beyond the far edge of zone  */
                s_sweep_dir = -s_sweep_dir;
                MotorCtrl_HomingCreepVel(s_sweep_dir, HOMING_PREC_VEL_RADS);
                (void)HwIo_GetProxRisingEdge();   /* flush before edge B */
                s_search_start_rad = pos;
                hom_dbg("OVS", rad_to_deg(s_edge_a), rad_to_deg(pos), rad_to_deg(pos));
                s_hom = HOM_FIND_EDGE_B;
            } else if (past > deg_to_rad(HOMING_OVERSHOOT_MAX_DEG)) {
                /* Sensor never went OFF — zone too wide or sensor stuck ON   */
#if !DISABLE_ALL_FAULTS
                RobotState.fsm              = STATE_FAULT;
                RobotState.comms.fault_code = 3u;
                MotorCtrl_Stop();
#endif
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_FIND_EDGE_B:
        /* Stage 3c — same precision vel as edge A; latency bias cancels in
           the (A+B)/2 midpoint. 100 Hz latch guarantees edge capture.      */
        if (HwIo_GetProxRisingEdge()) {
            s_edge_b = pos;
            float center = (s_edge_a + s_edge_b) * 0.5f;
            hom_dbg("EB1", rad_to_deg(s_edge_a), rad_to_deg(s_edge_b), rad_to_deg(pos));
            hom_dbg("CTR", rad_to_deg(center), rad_to_deg(fabsf(s_edge_b - s_edge_a)), rad_to_deg(pos));
            MotorCtrl_Stop();
            s_go_center_dir   = 0;
            s_center_start_ms = now;
            s_hom = HOM_GO_CENTER;
        } else if (fabsf(pos - s_search_start_rad) > deg_to_rad(HOMING_MAX_SEARCH_DEG)) {
            /* Safety: edge B not found within search range — fault        */
#if !DISABLE_ALL_FAULTS
            RobotState.fsm              = STATE_FAULT;
            RobotState.comms.fault_code = 2u;
            MotorCtrl_Stop();
#endif
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_GO_CENTER: {
        float center = (s_edge_a + s_edge_b) * 0.5f;
        float err    = center - pos;
        bool  close  = fabsf(err) <= deg_to_rad(0.1f);
        bool  timeout = (now - s_center_start_ms) >= HOMING_CENTER_TIMEOUT_MS;

        if (close || timeout) {
            MotorCtrl_Stop();
            hom_dbg("CEND", rad_to_deg(center), rad_to_deg(err), rad_to_deg(pos));
            s_settle_t = now;
            s_hom      = HOM_SETTLE;
        } else {
            int8_t needed = (err > 0.0f) ? +1 : -1;
            if (needed != s_go_center_dir) {
                s_go_center_dir = needed;
                MotorCtrl_HomingCreepVel(needed, HOMING_PREC_VEL_RADS); /* slow, accurate */
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case HOM_SETTLE:
        /* Wait for arm to fully stop, then define this position as home   */
        if (now - s_settle_t < HOMING_SETTLE_MS) break;

        {
            /* Base offset in degrees (sensor center → working 0°).
               Fine-tune: reg 0x32 in whole degrees (user-adjustable, default 0).  */
            float base_rad  = deg_to_rad(-HOME_OFFSET_DEG);
            int16_t adj_deg = (int16_t)ModbusBridge_GetReg(MODBUS_REG_HOME_OFFSET);
            s_home_offset_rad = base_rad + deg_to_rad((float)adj_deg) + deg_to_rad(TuningParams.homing.offset_deg);
            hom_dbg("ZERO", rad_to_deg((s_edge_a + s_edge_b) * 0.5f),
                    rad_to_deg(s_user_home_rad),
                    rad_to_deg(MotorCtrl_GetPosition_rad()));
            MotorCtrl_Zero(s_home_offset_rad);
            MotorCtrl_SetZvdBypass(true);   /* keep ZVD off until rod is attached and ZVD params are re-tuned */
            MotorCtrl_SetTarget(s_user_home_rad);
            Joystick_SendAudio('H'); /* @H = homing complete */
        }
        s_hom           = HOM_IDLE;
        s_move_start_ms = HAL_GetTick();
        RobotState.fsm     = STATE_RUNNING;
        s_run_mode      = RUN_POINT;
        set_task(0x0008);   /* GoPoint — drive to park position */
        break;

    case HOM_IDLE:
    default:
        break;
    }

}

/* =========================================================================
   AUTO sequence execution
   ========================================================================= */
static void auto_run(void)
{
    if (s_seq_pairs == 0 || s_seq_step > s_seq_pairs * 2u + 1u || s_seq_step > 17u) {
        /* Fallback / safety exit */
        RobotState.fsm         = STATE_IDLE;
        s_run_mode          = RUN_IDLE;
        set_task(0x0000);
        ModbusBridge_SetReg(0x22, 0);
        return;
    }

    /* Wait: motor must be at target AND gripper must be idle before next action */
    if (s_seq_step > 0u && (!MotorCtrl_IsAtTarget() || s_grip != GRIP_IDLE)) return;

    if (s_seq_step == s_seq_pairs * 2u + 1u) {
        snprintf(s_dbg, sizeof(s_dbg), "$AUTO,DONE,%u\r\n", s_seq_step);
        UartDma_SendTelemetry(s_dbg);
        RobotState.fsm         = STATE_IDLE;
        s_run_mode          = RUN_IDLE;
        s_gripper_triggered = false;
        set_task(0x0000);
        ModbusBridge_SetReg(0x22, 0); /* Clear pairs so next tab switch doesn't auto-start */
        return;
    }

    /* Motor arrived. Start gripper sequence for this position (once per arrival).
       Odd step = just arrived at pick; even non-zero step = just arrived at place.
       Re-read gripper enable here to handle race where 0x04 arrives after mode
       arm (dashboard writes mode=4 before gripper_auto_en=1).                  */
    s_gripper_en = (ModbusBridge_GetReg(0x04) & 0x01u) != 0u;
    if (s_seq_step > 0u && s_gripper_en && !s_gripper_triggered) {
        gripper_start((s_seq_step % 2u) == 1u); /* true=pick, false=place */
        s_gripper_triggered = true;
        return; /* re-enter when gripper finishes */
    }
    s_gripper_triggered = false;

    /* Command next motor move */
    float target;
    if (s_seq_step == s_seq_pairs * 2u) {
        /* Final move: return to the nearest home (nearest 360 degree multiple) 
           to prevent the robot from violently unwinding backwards to 0.0! */
        float current_deg = rad_to_deg(MotorCtrl_GetPosition_rad());
        float nearest_360 = roundf(current_deg / 360.0f) * 360.0f;
        target = deg_to_rad(nearest_360);
        set_task(0x0008u); /* GoPoint */
        snprintf(s_dbg, sizeof(s_dbg), "$AUTO,%u,HOME,%ld\r\n", s_seq_step, (long)nearest_360);
    } else {
        uint8_t pair   = s_seq_step / 2u;
        bool    pick   = (s_seq_step & 1u) == 0u;
        int16_t raw    = s_seq_slots[pair * 2u + (pick ? 0u : 1u)];
        /* Base system leaves 0x23 at 0 (degrees), but slots are indices!
           Force unit=1 to ensure proper 5° multiplication. */
        target = resolve_target(raw, 1);
        set_task(pick ? 0x0002u : 0x0004u);

        /* Log each commanded move: $AUTO,step,raw_slot,target_deg×10             */
        snprintf(s_dbg, sizeof(s_dbg), "$AUTO,%u,%d,%ld\r\n",
                 s_seq_step,
                 (int)raw,
                 (long)lroundf(target * (180.0f / M_PI) * 10.0f));
    }

    /* Step 0: sync S-curve from actual position before first move so the
       outer-loop deadband does not swallow the first few position-PID ticks.
       After homing/park the S-curve's internal pos matches reality, but a
       SyncTrajectory call guarantees it regardless of any preceding stop.     */
    if (s_seq_step == 0u) {
        MotorCtrl_SyncTrajectory();   /* re-seeds S-curve + clears integrals  */
    }

    UartDma_SendTelemetry(s_dbg);
    MotorCtrl_SetTarget(target);
    s_seq_step++;
}

/* =========================================================================
   Joystick handler — called every App_Run from main loop

   Two jog implementations are provided.  Activate exactly one by defining
   JOG_OPTION_1 (velocity bypass) or JOG_OPTION_2 (aggressive step mode).
   The default is Option 1.  Comment/uncomment in params.h or here.

   OPTION 1 — Velocity Bypass (preferred):
     L/R held → MotorCtrl_JogVelocity() called every App_Run.
     FSM stays IDLE.  The outer position loop is bypassed entirely; the inner
     1 kHz velocity PID tracks the joystick velocity directly.
     Released → MotorCtrl_JogRelease() re-engages the position loop without
     a bump by inheriting the current Kalman velocity as the S-curve initial
     condition and flushing the ZVD buffer to the lock-on position.

   OPTION 2 — Aggressive Step Mode:
     L/R held → each press dispatches a discrete SetTarget(pos + step) but
     with ZVD bypassed and S-curve Amax/Jmax raised 5–6× so each step
     completes in < one App_Run period.  The auto-repeat that previously
     caused ZVD buffer interference is now harmless because the previous
     step is always finished before the next one is queued.
   ========================================================================= */

/* Select Option 1 by default.  To use Option 2 comment this out. */
#define JOG_OPTION_1

static void handle_joystick(void)
{
    static bool s_f_pressed = false;

    if (!s_joy_mode) {
        /* Mode switched away — release joystick velocity jog cleanly.
           Guard: do NOT cancel during STATE_RUNNING (PC discrete jog owns
           s_jog_vel_active in that state and calls JogRelease itself).        */
        if (RobotState.fsm != STATE_RUNNING) {
#ifdef JOG_OPTION_1
            if (s_jog_vel_active)  { MotorCtrl_JogRelease();       s_jog_vel_active  = false; }
#else
            if (s_jog_step_active) { MotorCtrl_JogStepDisengage(); s_jog_step_active = false; }
#endif
        }
        return;
    }

    JoyState_t joy = Joystick_GetState();
    if (joy.base != 'F') {
        s_f_pressed = false;
    }

    if (!joy.connected || !Joystick_IsAlive()) {
        /* Gamepad lost mid-jog — release cleanly so motor does not coast */
#ifdef JOG_OPTION_1
        if (s_jog_vel_active)  { MotorCtrl_JogRelease();       s_jog_vel_active  = false; }
#else
        if (s_jog_step_active) { MotorCtrl_JogStepDisengage(); s_jog_step_active = false; }
#endif
#if !DISABLE_ALL_FAULTS
        if (RobotState.fsm != STATE_FAULT) {
            MotorCtrl_Stop();
            RobotState.fsm = STATE_FAULT;
            RobotState.comms.fault_code = 0x11u; /* 0x11 for joystick connection lost */
            s_run_mode = RUN_IDLE;
            set_task(0x0000);
        }
#endif
        return;
    }

    /* Emergency — release jog first so the FSM fault path sees a stopped motor */
    if (joy.emergency == 'P' || joy.base == 'X') {
#ifdef JOG_OPTION_1
        if (s_jog_vel_active)  { MotorCtrl_JogRelease();       s_jog_vel_active  = false; }
#else
        if (s_jog_step_active) { MotorCtrl_JogStepDisengage(); s_jog_step_active = false; }
#endif
#if !DISABLE_ALL_FAULTS
        if (RobotState.fsm != STATE_FAULT) {
            MotorCtrl_Stop();
            RobotState.fsm              = STATE_FAULT;
            RobotState.comms.fault_code = 0x10u;
            s_run_mode               = RUN_IDLE;
            set_task(0x0000);
        }
#endif
        return;
    }

/* ---- Option 1: Velocity Bypass -----------------------------------------*/
#ifdef JOG_OPTION_1

    if (joy.base == 'L' || joy.base == 'R') {
        /* FSM must be IDLE and gripper quiet.  Do not enter during homing,
           fault, or running — those states own the motor.                   */
        if (RobotState.fsm != STATE_IDLE) { return; }
        if (s_grip != GRIP_IDLE)       { return; }

        /* JOY_JOG_VEL_RADS is the continuous speed while the key is held.
           MotorCtrl_JogVelocity() clamps to ±Vmax internally.              */
        float vel = (joy.base == 'R') ?  JOY_JOG_VEL_RADS
                                       : -JOY_JOG_VEL_RADS;
        MotorCtrl_JogVelocity(vel);
        s_jog_vel_active = true;
        /* FSM intentionally stays IDLE — no position target, no move timeout */
        return;
    }

    /* Key is not L/R.  If velocity jog was active, release it now.          */
    if (s_jog_vel_active) {
        MotorCtrl_JogRelease();
        s_jog_vel_active = false;
        /* Fall through: process any non-jog button in the same App_Run.     */
    }

    /* Non-jog buttons — only accepted in IDLE with gripper stopped          */
    if (RobotState.fsm != STATE_IDLE) { return; }
    if (s_grip != GRIP_IDLE)       { return; }

    switch (joy.base) {
    case 'A':  gripper_start(true);       break;  /* Pick sequence            */
    case 'B':  gripper_start(false);      break;  /* Place sequence           */
    case 'U':  Gripper_SetVertical(true); break;  /* Manual arm up            */
    case 'D':  Gripper_SetVertical(false);break;  /* Manual arm down          */
    case 'F':
        if (!s_f_pressed) {
            bool current_open = (HAL_GPIO_ReadPin(Gripper_Down_GPIO_Port, Gripper_Down_Pin) == GPIO_PIN_RESET);
            Gripper_SetClaw(!current_open);
            s_f_pressed = true;
        }
        break;
    case 'Y':
        RobotState.fsm = STATE_HOMING;
        s_hom       = HOM_INIT;
        set_task(0x0001);
        Joystick_SendAudio('h');
        break;
    default: break;
    }

/* ---- Option 2: Aggressive Step Mode ------------------------------------*/
#else /* JOG_OPTION_2 */

    /* Detect joystick release: key is not L/R but step mode was engaged.
       This runs every App_Run while FSM is IDLE, so it fires as soon as the
       last step completes and the key has already been released.             */
    if (joy.base != 'L' && joy.base != 'R' && s_jog_step_active) {
        MotorCtrl_JogStepDisengage();
        s_jog_step_active = false;
    }

    /* All commands only accepted in IDLE with gripper stopped               */
    if (RobotState.fsm != STATE_IDLE) { return; }
    if (s_grip != GRIP_IDLE)       { return; }

    switch (joy.base) {
    case 'L':
    case 'R': {
        /* Engage step mode on first press of this hold-to-repeat sequence.
           Subsequent passes keep the mode active until release.             */
        if (!s_jog_step_active) {
            MotorCtrl_JogStepEngage();
            s_jog_step_active = true;
        }
        /* Dispatch a discrete step.  With ZVD bypassed and Amax/Jmax raised,
           this step will complete in < 80 ms so the trajectory is done
           before handle_joystick() is called again and fires the next step. */
        float step = deg_to_rad((joy.base == 'R') ?  JOY_JOG_STEP_DEG
                                                    : -JOY_JOG_STEP_DEG);
        MotorCtrl_SetTarget(MotorCtrl_GetPosition_rad() + step);
        set_task(0x0008);
        s_move_start_ms = HAL_GetTick();
        RobotState.fsm     = STATE_RUNNING;
        s_run_mode      = RUN_JOG;
        break;
    }
    case 'A':  gripper_start(true);        break;
    case 'B':  gripper_start(false);       break;
    case 'U':  Gripper_SetVertical(true);  break;
    case 'D':  Gripper_SetVertical(false); break;
    case 'F':
        if (!s_f_pressed) {
            bool current_open = (HAL_GPIO_ReadPin(Gripper_Down_GPIO_Port, Gripper_Down_Pin) == GPIO_PIN_RESET);
            Gripper_SetClaw(!current_open);
            s_f_pressed = true;
        }
        break;
    case 'Y':
        RobotState.fsm = STATE_HOMING;
        s_hom       = HOM_INIT;
        set_task(0x0001);
        Joystick_SendAudio('h');
        break;
    default: break;
    }

#endif /* JOG_OPTION_1 / JOG_OPTION_2 */
}

/* =========================================================================
   Main FSM
   ========================================================================= */
static void fsm_run(void)
{
    /* ── Two-stage E-stop: stop-and-verify ──────────────────────────────────
       Stage 1 (trigger):  debounced estop fires → motor stops IMMEDIATELY.
       Stage 2 (verify):   wait ESTOP_VERIFY_MS with motor off.  Motor EMI
                           disappears once PWM stops; a real button stays LOW.
         • Pin still LOW  → real E-stop  → latch STATE_FAULT (0x01).
         • Pin went HIGH  → was EMI noise → resume, no fault.              */
#if !DISABLE_ALL_FAULTS
    {
        static bool     s_estop_verifying  = false;
        static uint32_t s_estop_verify_t   = 0u;
        static FsmState_t s_estop_prev_fsm = STATE_INIT;

        if (s_estop_verifying) {
            /* Motor already stopped.  Wait for EMI to dissipate. */
            if (HAL_GetTick() - s_estop_verify_t >= ESTOP_VERIFY_MS) {
                /* Re-read the raw pin RIGHT NOW (bypass debouncer) */
                bool still_active =
                    (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin) == GPIO_PIN_RESET);
                if (still_active) {
                    /* Confirmed real E-stop press */
                    RobotState.fsm              = STATE_FAULT;
                    RobotState.comms.fault_code = 0x01u;
                    set_task(0x0000);
                } else {
                    /* Was EMI noise — restore previous FSM state */
                    RobotState.fsm = s_estop_prev_fsm;
                    HwIo_ResetEstopDebounce();
                }
                s_estop_verifying = false;
            }
            return;   /* don't process any other FSM logic while verifying */
        }

        if (!RobotState.dbg.estop_disabled &&
            RobotState.sensors.estop && RobotState.fsm != STATE_FAULT) {
            /* Stage 1: IMMEDIATELY stop motor for safety */
            MotorCtrl_Stop();
            s_estop_prev_fsm  = RobotState.fsm;
            s_estop_verifying = true;
            s_estop_verify_t  = HAL_GetTick();
            return;
        }
    }
#endif

    uint16_t mode_reg = ModbusBridge_GetReg(0x01);

    switch (RobotState.fsm) {

    case STATE_INIT:
        RobotState.fsm = STATE_IDLE;
        break;

    case STATE_IDLE:
        set_task(0x0000);
        if (mode_reg & 0x01u) {                         /* Home             */
            ModbusBridge_SetReg(0x01, 0);
            RobotState.fsm = STATE_HOMING;
            s_hom       = HOM_INIT;
            set_task(0x0001);
            s_hb_last_tick = HAL_GetTick(); /* HOME write proves PC alive — reset HB timer */
        } else if ((mode_reg & 0x02u) || (ModbusBridge_GetReg(0x05) != 0u)) { /* Jog step */
            int16_t step_raw = (int16_t)ModbusBridge_GetReg(0x05);
            if (step_raw != 0) {
                ModbusBridge_SetReg(0x01, 0);
                /* Clear trigger immediately — prevents re-firing on next App_Run.
                   PC writes a non-zero signed degree value (e.g. +5 or -10).
                   Firmware fires one discrete move and self-clears.              */
                ModbusBridge_SetReg(0x05, 0);

                /* Release any lingering velocity jog cleanly before step move    */
                if (s_jog_vel_active) {
                    MotorCtrl_JogRelease();
                    s_jog_vel_active = false;
                }

                /* Velocity-bypass jog: drive toward target at JOG_PC_VEL_RADS.
                   Bypasses position PID (which generates too small a velocity
                   command for fine steps).  RUN_JOG monitors position and calls
                   JogRelease() when within the deadband.                       */
                float step_rad = deg_to_rad((float)step_raw);
                s_jog_target_rad = MotorCtrl_GetPosition_rad() + step_rad;

                float dir = (step_raw > 0) ? 1.0f : -1.0f;
                MotorCtrl_JogVelocity(dir * JOG_PC_VEL_RADS);
                s_jog_vel_active = true;

                set_task(0x0008);           /* GoPoint                           */
                s_move_start_ms = HAL_GetTick();
                RobotState.fsm = STATE_RUNNING;
                s_run_mode  = RUN_JOG;
            }
        } else if (mode_reg & 0x04u) {                  /* Auto             */
            uint8_t pairs = (uint8_t)ModbusBridge_GetReg(0x22);
            if (pairs > 0u) {
                ModbusBridge_SetReg(0x01, 0); /* only consume mode when sequence is ready */
                ModbusBridge_SetReg(0x24, 0); /* clear stale p2p_target so it doesn't
                                                 fire a P2P move when auto finishes     */
                s_seq_pairs  = (pairs > 8u) ? 8u : pairs;
                s_gripper_en = (ModbusBridge_GetReg(0x04) & 0x01u) != 0u;
                for (uint8_t i = 0; i < 16u; i++) {
                    s_seq_slots[i] = (int16_t)ModbusBridge_GetReg(0x12u + i);
                }
                s_seq_step          = 0;
                s_gripper_triggered = false;
                RobotState.fsm = STATE_RUNNING;
                s_run_mode  = RUN_AUTO;
            }

        } else if (mode_reg & 0x08u) {                  /* SetHome          */
            ModbusBridge_SetReg(0x01, 0);
            /* Zero the encoder at the current physical position immediately.
               Arm must be stopped (STATE_IDLE). After this, pos reads 0°
               and all subsequent moves reference this as home.              */
            MotorCtrl_Zero(0.0f);
            MotorCtrl_SetTarget(0.0f);
            s_user_home_rad = 0.0f;
        } else if (mode_reg & 0x10u) {                  /* Test  */
            ModbusBridge_SetReg(0x01, 0);
            s_test_type           = ModbusBridge_GetReg(0x06);
            s_test_pos_a          = (int16_t)ModbusBridge_GetReg(0x09);
            s_test_pos_b          = (int16_t)ModbusBridge_GetReg(0x10);
            s_test_repeats_target = ModbusBridge_GetReg(0x11);
            s_test_repeats_done   = 0;
            s_test_moving_to_b    = true;
            s_test_dwell_start    = 0;

            if (s_test_type == 1) { /* Performance */
                int16_t raw_v = (int16_t)ModbusBridge_GetReg(0x07);
                int16_t raw_a = (int16_t)ModbusBridge_GetReg(0x08);
                if (raw_v > 0) TuningParams.scurve.vmax_rads = (float)raw_v;
                if (raw_a > 0) TuningParams.scurve.amax_rads2 = (float)raw_a;
            }

            uint16_t unit = ModbusBridge_GetReg(0x23);
            MotorCtrl_SetTarget(resolve_target(s_test_pos_b, unit));
            
            set_task(0x0008); /* GoPoint */
            s_move_start_ms = HAL_GetTick();
            RobotState.fsm = STATE_RUNNING;
            s_run_mode  = RUN_TEST;
        }

        /* Soft stop clears running state */
        if (ModbusBridge_GetReg(0x25) & 0x01u) {
            MotorCtrl_Stop();
        }

        /* Point-to-point command */
        {
            uint16_t p2p_unit = ModbusBridge_GetReg(0x23);
            int16_t  p2p_tgt  = (int16_t)ModbusBridge_GetReg(0x24);
            if (p2p_tgt != 0) {
                ModbusBridge_SetReg(0x24, 0);
                MotorCtrl_SetTarget(resolve_target(p2p_tgt, p2p_unit));
                set_task(0x0008); /* GoPoint */
                s_move_start_ms = HAL_GetTick();
                RobotState.fsm = STATE_RUNNING;
                s_run_mode  = RUN_POINT;
            }
        }

        /* Gripper manual pick/place sequence (0x03): 1=Pick, 2=Place */
        {
            uint16_t grip_cmd = ModbusBridge_GetReg(0x03);
            if (grip_cmd != 0u && s_grip == GRIP_IDLE) {
                ModbusBridge_SetReg(0x03, 0);
                if (grip_cmd & 0x01u)      gripper_start(true);  /* Pick  */
                else if (grip_cmd & 0x02u) gripper_start(false); /* Place */
            }
        }
        break;

    case STATE_HOMING:
        homing_run();
        break;

    case STATE_RUNNING:
        /* Soft stop */
        if (ModbusBridge_GetReg(0x25) & 0x01u) {
            MotorCtrl_Stop();
            RobotState.fsm = STATE_IDLE;
            s_run_mode  = RUN_IDLE;
            ModbusBridge_SetReg(0x22, 0); /* Clear auto sequence state */
            
            /* Restore TuningParams if interrupted during Performance Test */
            TuningParams.scurve.vmax_rads = SCURVE_VMAX_RADS;
            TuningParams.scurve.amax_rads2 = SCURVE_AMAX_RADS2;
            break;
        }
        switch (s_run_mode) {
        case RUN_JOG: {
            /* Velocity-bypass jog: drive toward s_jog_target_rad at fixed
               speed.  Stop when within the position deadband.                 */
            float pos = MotorCtrl_GetPosition_rad();
            float err = s_jog_target_rad - pos;

            if (fabsf(err) <= POSITION_DEADBAND_RAD) {
                MotorCtrl_JogRelease();
                s_jog_vel_active = false;
                RobotState.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            } else if ((HAL_GetTick() - s_move_start_ms) >= MOVE_TIMEOUT_MS) {
                MotorCtrl_JogRelease();
                s_jog_vel_active = false;
                RobotState.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            } else {
                float dir = (err > 0.0f) ? 1.0f : -1.0f;
                MotorCtrl_JogVelocity(dir * JOG_PC_VEL_RADS);
            }
            break;
        }
        case RUN_POINT:
            if (MotorCtrl_IsAtTarget()) {
                RobotState.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            } else if ((HAL_GetTick() - s_move_start_ms) >= MOVE_TIMEOUT_MS) {
                MotorCtrl_Stop();
                RobotState.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            }
            break;

        case RUN_TEST:
            if (MotorCtrl_IsAtTarget()) {
                if (s_test_dwell_start == 0) {
                    s_test_dwell_start = HAL_GetTick();
                } else if ((HAL_GetTick() - s_test_dwell_start) >= 250u) { /* 250ms dwell */
                    s_test_dwell_start = 0;
                    
                    if (s_test_moving_to_b) {
                        s_test_moving_to_b = false;
                        MotorCtrl_SetTarget(resolve_target(s_test_pos_a, ModbusBridge_GetReg(0x23)));
                        s_move_start_ms = HAL_GetTick();
                    } else {
                        s_test_moving_to_b = true;
                        s_test_repeats_done++;
                        /* 65000 treated as infinite */
                        if (s_test_repeats_target < 65000u && s_test_repeats_done >= s_test_repeats_target) {
                            RobotState.fsm = STATE_IDLE;
                            s_run_mode  = RUN_IDLE;
                            set_task(0x0000);
                            
                            if (s_test_type == 1) { /* Restore defaults */
                                TuningParams.scurve.vmax_rads = SCURVE_VMAX_RADS;
                                TuningParams.scurve.amax_rads2 = SCURVE_AMAX_RADS2;
                            }
                            break;
                        }
                        MotorCtrl_SetTarget(resolve_target(s_test_pos_b, ModbusBridge_GetReg(0x23)));
                        s_move_start_ms = HAL_GetTick();
                    }
                }
            } else if ((HAL_GetTick() - s_move_start_ms) >= MOVE_TIMEOUT_MS) {
                MotorCtrl_Stop();
                RobotState.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
                
                if (s_test_type == 1) {
                    TuningParams.scurve.vmax_rads = SCURVE_VMAX_RADS;
                    TuningParams.scurve.amax_rads2 = SCURVE_AMAX_RADS2;
                }
            }
            break;
        case RUN_AUTO:
            auto_run();
            break;
        default:
            RobotState.fsm = STATE_IDLE;
            break;
        }
        break;

    case STATE_FAULT:
        /* Latch — clear only if E-stop released AND reset button pressed.
           If estop_disabled is set, E-stop check is skipped — reset btn alone clears. */
        if ((RobotState.dbg.estop_disabled || !RobotState.sensors.estop) && HwIo_GetResetBtn()) {
            RobotState.fsm                          = STATE_IDLE;
            RobotState.comms.fault_code             = 0u;
            RobotState.dbg.safety.tripped_encoder   = false;
            RobotState.dbg.safety.tripped_boundary  = false;
            RobotState.dbg.safety.tripped_current   = false;
            RobotState.dbg.safety.tripped_tracking  = false;
        }
        break;
    }
}

/* =========================================================================
   Public entry points
   ========================================================================= */

void Dashboard_ParseCommand(const char* cmd)
{
    /* Acknowledge receipt of the dashboard command */
    snprintf(s_dbg, sizeof(s_dbg), "$DASH,ACK,%.30s\r\n", cmd);
    UartDma_SendTelemetry(s_dbg);

    /* Very basic scaffolding for the dashboard commands */
    if (strncmp(cmd, "LAB1_DECEL", 10) == 0) {
        if (RobotState.fsm == STATE_IDLE) {
            MotorCtrl_SetTarget(MotorCtrl_GetPosition_rad() + deg_to_rad(360.0f));
            set_task(0x0008); 
            s_move_start_ms = HAL_GetTick();
            RobotState.fsm = STATE_RUNNING;
            s_run_mode = RUN_POINT;
        }
    } else if (strncmp(cmd, "LAB2_STEP", 9) == 0) {
        if (RobotState.fsm == STATE_IDLE) {
            MotorCtrl_SetTarget(MotorCtrl_GetPosition_rad() + deg_to_rad(90.0f));
            set_task(0x0008); 
            s_move_start_ms = HAL_GetTick();
            RobotState.fsm = STATE_RUNNING;
            s_run_mode = RUN_POINT;
        }
    } else if (strncmp(cmd, "LAB4_CYCLE", 10) == 0) {
        if (RobotState.fsm == STATE_IDLE) {
            ModbusBridge_SetReg(0x22, 1); /* 1 pair */
            s_seq_slots[0] = 1;
            s_seq_slots[1] = 2;
            s_seq_pairs = 1;
            s_seq_step = 0;
            s_gripper_triggered = false;
            RobotState.fsm = STATE_RUNNING;
            s_run_mode = RUN_AUTO;
        }
    }
}


void App_Init(void)
{
    HwIo_Init();
    UartDma_Init();
    Joystick_Init();
    CanBus_Init();
    MotorCtrl_Init();
    ModbusBridge_Init();
    Motor_Enable();
    MotorCtrl_SetZvdBypass(false);  /* Rod is attached, ensure ZVD is enabled by default */
    HAL_TIM_Base_Start_IT(&htim6);
    RobotState.fsm         = STATE_INIT;
    s_hom               = HOM_IDLE;
    s_run_mode          = RUN_IDLE;
    s_grip              = GRIP_IDLE;
    s_gripper_triggered = false;

    /* Initialise mode relay to match physical switch at power-on */
    s_joy_mode = HwIo_GetSelectedMode();
    Relay_SetSysmode(s_joy_mode);

    /* Safety guards — all ON by default; set en_* = false via debugger to disable */
    RobotState.dbg.safety.en_encoder_health  = false;
    RobotState.dbg.safety.en_current_safety  = false;
    RobotState.dbg.safety.en_tracking_safety = false;
    RobotState.dbg.last_fault                = 0u;
    /* E-stop bypass — set to true in Live Expressions to disable ALL E-stop checks */
    RobotState.dbg.estop_disabled            = false;

    /* Seed heartbeat tracker — gives 2 s window before timeout is enforced */
    s_hb_last_tick = HAL_GetTick();

    /* IWDG: LSI ≈ 32 kHz, PR=÷16 → 2 kHz → 100 counts = 50 ms timeout.
       Refreshed every 1 ms from TIM6 ISR.  Kicks reset if control loop hangs. */
    IWDG->KR  = 0xCCCCu;   /* start IWDG                        */
    IWDG->KR  = 0x5555u;   /* unlock PR / RLR write access      */
    IWDG->PR  = 2u;        /* prescaler ÷16                     */
    IWDG->RLR = 99u;       /* 100 × 0.5 ms = 50 ms timeout      */
    IWDG->KR  = 0xAAAAu;   /* initial reload                    */
#if defined(DEBUG)
    SET_BIT(DBGMCU->APB1FZR1, DBGMCU_APB1FZR1_DBG_IWDG_STOP);
#endif
}

void App_Run(void)
{
    /* Poll sensors into RobotState (non-ISR sensors) */
    /* E-stop: NC contact on PA5 — polarity inverted in hw_io.c (HIGH=active) */
    RobotState.sensors.estop         = HwIo_GetEStop();
    RobotState.sensors.selected_mode = HwIo_GetSelectedMode();
    RobotState.sensors.reset_btn     = HwIo_GetResetBtn();
    RobotState.sensors.proximity     = HwIo_GetProximity();

    /* Mode switch: only act when the arm is idle. Ignore during FAULT (E-stop
       transients), RUNNING (jog/auto), and HOMING — motor direction reversals
       inject PWM EMI onto PA6 that can fake a mode flip, clicking the Sysmode
       relay + pilot lamp mid-motion. Deferring until IDLE means the change
       takes effect only once motion has fully settled.                        */
    if (RobotState.sensors.selected_mode != s_joy_mode
        && RobotState.fsm != STATE_FAULT
        && RobotState.fsm != STATE_RUNNING
        && RobotState.fsm != STATE_HOMING) {
        /* Release any active jog before switching modes so the motor stops
           cleanly and all integrals are reset regardless of which option is active */
        if (s_jog_vel_active)  { MotorCtrl_JogRelease();       s_jog_vel_active  = false; }
        if (s_jog_step_active) { MotorCtrl_JogStepDisengage(); s_jog_step_active = false; }
        s_joy_mode = RobotState.sensors.selected_mode;
        Relay_SetSysmode(s_joy_mode);
        HwIo_ResetEstopDebounce();  /* discard relay-induced spike on PA5 */
        Joystick_SendAudio(s_joy_mode ? 'J' : 'S');
    }

    /* Heartbeat timeout — soft-stop to IDLE if PC link silent ≥ 2 s */
    {
        uint16_t hb_now = ModbusBridge_GetReg(0x00);
        if (hb_now == HB_HI) {
            s_hb_last_tick = HAL_GetTick();
            ModbusBridge_SetReg(0x00, HB_YA); /* Reset to YA immediately so PC/main.exe can see it and reply again */
            /* Link restored — clear the stale PC-link-lost code */
            if (RobotState.comms.fault_code == 0x20u)
                RobotState.comms.fault_code = 0u;
        }
        uint32_t hb_age = HAL_GetTick() - s_hb_last_tick;
        RobotState.dbg.hb_age_ms = (hb_age > 0xFFFFu) ? 0xFFFFu : (uint16_t)hb_age;
        /* Do not interrupt homing — it has its own safety limits (FAULT codes 2-4).
           Stopping mid-sweep corrupts edge detection and causes 0.4°/12s creep. */
#if !DISABLE_ALL_FAULTS
        if (hb_age >= HEARTBEAT_TIMEOUT_MS &&
            RobotState.fsm != STATE_FAULT    &&
            RobotState.fsm != STATE_HOMING) {
            MotorCtrl_Stop();
            RobotState.fsm              = STATE_IDLE;
            s_run_mode               = RUN_IDLE;
            RobotState.comms.fault_code = 0x20u; /* PC Link Lost */
            set_task(0x0000);
        }
#endif
    }

    /* Update debug mirror — expand RobotState in Live Expressions to see all */
    {
        JoyState_t _j = Joystick_GetState();
        RobotState.dbg.run_mode = (uint8_t)s_run_mode;
        RobotState.dbg.grip     = (uint8_t)s_grip;
        RobotState.dbg.joy_mode = (uint8_t)s_joy_mode;
        RobotState.dbg.joy_btn  = _j.base;
        RobotState.dbg.joy_conn = (uint8_t)_j.connected;
        RobotState.dbg.pos_deg  = rad_to_deg(MotorCtrl_GetPosition_rad());
        RobotState.dbg.vel_dps  = RobotState.motion.velocity_rps * 360.0f;
        if (RobotState.comms.fault_code != 0u) {
            RobotState.dbg.last_fault = RobotState.comms.fault_code;
        }
    }

    /* Drain UART RX → Modbus callbacks → register updates */
    UartDma_Process();

    /* Refresh Modbus read registers */
    ModbusBridge_Tick();

    /* Tick CAN bus interface */
    CanBus_Tick();

    /* Advance gripper sequence (runs independently of FSM) */
    gripper_seq_run();

    /* Handle joystick before FSM so commands take effect this iteration */
    handle_joystick();

    /* Run main FSM */
    fsm_run();

    /* Emergency pilot lamp — ON whenever robot is in FAULT */
    Relay_SetStatus(RobotState.fsm == STATE_FAULT);

    /* Audio feedback on FSM state transitions */
    {
        static FsmState_t s_prev_fsm = STATE_INIT;
        if (RobotState.fsm != s_prev_fsm) {
            switch (RobotState.fsm) {
            case STATE_HOMING: Joystick_SendAudio('h'); break; /* homing started */
            case STATE_FAULT:  Joystick_SendAudio('E'); break; /* fault          */
            case STATE_IDLE:
                if (s_prev_fsm == STATE_FAULT) Joystick_SendAudio('C'); /* cleared */
                break;
            default: break;
            }
            s_prev_fsm = RobotState.fsm;
        }
    }

    /* Periodic FSM state telemetry every 2 s for debug.
       Format: $ST,<FsmState_t>,<RunMode_t>  (0=INIT,1=HOMING,2=IDLE,3=RUNNING,4=FAULT) */
    uint32_t now = HAL_GetTick();
    if (now - s_tel_tick >= 2000u) {
        s_tel_tick = now;
        snprintf(s_dbg, sizeof(s_dbg), "$ST,%d,%d,%d\r\n",
                 (int)RobotState.fsm, (int)s_run_mode,
                 (int)RobotState.comms.fault_code);
        UartDma_SendTelemetry(s_dbg);
    }
}

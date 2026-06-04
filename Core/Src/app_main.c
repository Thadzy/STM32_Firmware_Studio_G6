#include "app_main.h"
#include "modbus_bridge.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
#include "joystick.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim6;

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* =========================================================================
   P2P index position table
   Fill in physical positions (degrees) once hardware layout is finalised.
   Index 1 → g_idx_deg[0], index 2 → g_idx_deg[1], …
   ========================================================================= */
static float g_idx_deg[P2P_INDEX_COUNT] = {
    0.0f, 36.0f, 72.0f, 108.0f, 144.0f,   /* TODO: measure on hardware     */
    180.0f, 216.0f, 252.0f, 288.0f, 324.0f
};

/* =========================================================================
   Homing sub-state machine
   Motion law: ALWAYS HomingCreep (HOMING_VEL_RADS). Never S-curve during home.

   Sequence:
     SWEEP      — triangular sweep with growing amplitude until sensor fires (rising edge = A)
     OVERSHOOT  — continue HOMING_OVERSHOOT_DEG past edge A to guarantee sensor is cleared
     FIND_EDGE_B — reverse; rising edge from other side = edge B
     GO_CENTER  — creep to (A + B) / 2; timeout after HOMING_CENTER_TIMEOUT_MS
     SETTLE     — hold still for HOMING_SETTLE_MS then zero
   ========================================================================= */
typedef enum {
    HOM_IDLE = 0,
    HOM_INIT,
    HOM_SWEEP,        /* growing-amplitude sweep until sensor ON = edge A    */
    HOM_OVERSHOOT,    /* continue past edge A to clear the sensor            */
    HOM_FIND_EDGE_B,  /* reversed — rising edge from other side = edge B     */
    HOM_GO_CENTER,    /* creep to (A + B) / 2                                */
    HOM_SETTLE,       /* wait HOMING_SETTLE_MS then zero                     */
} HomingState_t;

static HomingState_t s_hom;
static float    s_sweep_start_rad;  /* position at start of current half-cycle */
static int8_t   s_sweep_dir;        /* +1 = forward, -1 = reverse              */
static float    s_sweep_amp_rad;    /* current half-cycle amplitude             */
static float    s_edge_a;           /* rising edge entering sensor from sweep direction       */
static float    s_edge_b;           /* rising edge entering sensor from opposite direction    */
static float    s_search_start_rad; /* position when FIND_EDGE_B begins (for safety limit)   */
static uint32_t s_settle_t;
static uint32_t s_center_start_ms; /* HAL_GetTick when GO_CENTER begins — for timeout        */
static float    s_home_offset_rad;
static float    s_user_home_rad;    /* park position saved by SetHome (post-calibration frame) */
static int8_t   s_go_center_dir;    /* tracks last dir in HOM_GO_CENTER — avoids repeated HomingCreep calls */
static uint32_t s_move_start_ms;   /* HAL_GetTick() when STATE_RUNNING was entered — move timeout ref */
static uint32_t s_tel_tick;        /* last $ST telemetry send time */

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

/* Auto sequence state */
static uint8_t  s_seq_pairs;            /* number of pick+place pairs (max 8)    */
static int16_t  s_seq_slots[16];        /* registers 0x12–0x21 = 16 slots = 8 pairs */
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

/* Resolve P2P target to radians given unit and signed raw value             */
static float resolve_target(int16_t raw, uint16_t unit)
{
    if (unit == 0u) {
        /* degree mode — raw is signed degrees */
        return deg_to_rad((float)raw);
    } else {
        /* index mode — |raw|-1 indexes into table; sign = direction hint    */
        int16_t idx = (raw < 0) ? -raw : raw;
        if (idx < 1 || idx > (int16_t)P2P_INDEX_COUNT) return 0.0f;
        float target_deg = g_idx_deg[idx - 1];
        return deg_to_rad(target_deg);
    }
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

    switch (s_hom) {

    /* ------------------------------------------------------------------ */
    case HOM_INIT:
        s_sweep_start_rad = pos;
        s_sweep_dir       = +1;
        s_sweep_amp_rad   = deg_to_rad(HOMING_WIGGLE_STEP_DEG); /* first step */
        s_edge_a          = 0.0f;
        s_edge_b          = 0.0f;
        (void)HwIo_GetProxRisingEdge();   /* flush any stale latch from before homing */

        /* Whether inside or outside the zone, always start the sweep.
           If inside, the sweep exits first then catches the proper rising
           edge on re-entry — giving a real edge A, not just parked position. */
        MotorCtrl_HomingCreep(s_sweep_dir);
        s_hom = HOM_SWEEP;
        break;

    /* ------------------------------------------------------------------ */
    case HOM_SWEEP:
        /* Sinusoidal (triangular) sweep: creep at constant slow speed.
           Each time the arm travels one amplitude in the current direction,
           reverse and grow the amplitude — like a growing sine envelope.  */
        {
            float traveled = (pos - s_sweep_start_rad) * (float)s_sweep_dir;

            if (traveled >= s_sweep_amp_rad) {
                /* End of half-cycle — reverse and grow */
                s_sweep_dir       = -s_sweep_dir;
                s_sweep_start_rad = pos;
                s_sweep_amp_rad  += deg_to_rad(HOMING_WIGGLE_STEP_DEG);

                if (s_sweep_amp_rad > deg_to_rad(HOMING_WIGGLE_MAX_DEG)) {
                    g_robot.fsm              = STATE_FAULT;
                    g_robot.comms.fault_code = 4u; /* sensor not found in sweep */
                    MotorCtrl_Stop();
                    break;
                }
                MotorCtrl_HomingCreep(s_sweep_dir);
            }

            /* Rising edge — proximity just turned ON → edge A found       */
            if (HwIo_GetProxRisingEdge()) {
                s_edge_a = pos;
                hom_dbg("EA1", rad_to_deg(pos), rad_to_deg(s_sweep_amp_rad), rad_to_deg(pos));
                s_hom = HOM_OVERSHOOT;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_OVERSHOOT:
        /* Continue past edge A until the sensor physically clears (prox OFF).
           A fixed 5-deg stop is not enough when the sensor window is wider
           than HOMING_OVERSHOOT_DEG — the motor would reverse while still
           inside the zone, exit from the wrong side (A), and never find B.  */
        {
            float past = (pos - s_edge_a) * (float)s_sweep_dir;
            if (past >= deg_to_rad(HOMING_OVERSHOOT_DEG) && !HwIo_GetProximity()) {
                /* Sensor cleared — motor is now beyond the far edge of zone  */
                s_sweep_dir = -s_sweep_dir;
                MotorCtrl_HomingCreep(s_sweep_dir);
                (void)HwIo_GetProxRisingEdge();   /* flush latch */
                s_search_start_rad = pos;
                hom_dbg("OVS", rad_to_deg(s_edge_a), rad_to_deg(pos), rad_to_deg(pos));
                s_hom = HOM_FIND_EDGE_B;
            } else if (past > deg_to_rad(HOMING_OVERSHOOT_MAX_DEG)) {
                /* Sensor never went OFF — zone too wide or sensor stuck ON   */
                g_robot.fsm              = STATE_FAULT;
                g_robot.comms.fault_code = 3u;
                MotorCtrl_Stop();
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_FIND_EDGE_B:
        /* 100 Hz latch guarantees edge detection even if App_Run is slow  */
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
            g_robot.fsm              = STATE_FAULT;
            g_robot.comms.fault_code = 2u;
            MotorCtrl_Stop();
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_GO_CENTER: {
        float center = (s_edge_a + s_edge_b) * 0.5f;
        float err    = center - pos;
        bool  close  = fabsf(err) <= deg_to_rad(0.5f);
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
                MotorCtrl_HomingCreep(needed);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case HOM_SETTLE:
        /* Wait for arm to fully stop, then define this position as home   */
        if (now - s_settle_t < HOMING_SETTLE_MS) break;

        {
            int16_t off_raw   = (int16_t)ModbusBridge_GetReg(MODBUS_REG_HOME_OFFSET);
            s_home_offset_rad = deg_to_rad((float)off_raw);
            hom_dbg("ZERO", rad_to_deg((s_edge_a + s_edge_b) * 0.5f),
                    rad_to_deg(s_user_home_rad),
                    rad_to_deg(MotorCtrl_GetPosition_rad()));
            MotorCtrl_Zero(s_home_offset_rad);
            MotorCtrl_SetTarget(s_user_home_rad);
            Joystick_SendAudio('H'); /* @H = homing complete */
        }
        s_hom           = HOM_IDLE;
        s_move_start_ms = HAL_GetTick();
        g_robot.fsm     = STATE_RUNNING;
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
    if (s_seq_pairs == 0 || s_seq_step >= s_seq_pairs * 2u || s_seq_step >= 16u) {
        g_robot.fsm         = STATE_IDLE;
        s_run_mode          = RUN_IDLE;
        s_gripper_triggered = false;
        set_task(0x0000);
        return;
    }

    /* Wait: motor must be at target AND gripper must be idle before next action */
    if (s_seq_step > 0u && (!MotorCtrl_IsAtTarget() || s_grip != GRIP_IDLE)) return;

    /* Motor arrived. Start gripper sequence for this position (once per arrival).
       Odd step = just arrived at pick; even non-zero step = just arrived at place. */
    if (s_seq_step > 0u && s_gripper_en && !s_gripper_triggered) {
        gripper_start((s_seq_step % 2u) == 1u); /* true=pick, false=place */
        s_gripper_triggered = true;
        return; /* re-enter when gripper finishes */
    }
    s_gripper_triggered = false;

    /* Command next motor move */
    uint8_t pair   = s_seq_step / 2u;
    bool    pick   = (s_seq_step & 1u) == 0u;
    int16_t raw    = s_seq_slots[pair * 2u + (pick ? 0u : 1u)];
    float   target = resolve_target(raw, 1u); /* sequence always uses index */

    set_task(pick ? 0x0002u : 0x0004u);
    MotorCtrl_SetTarget(target);
    s_seq_step++;
}

/* =========================================================================
   Joystick handler — called every App_Run from main loop
   L/R auto-repeat: ESP32 sends state only on change; when the motor finishes
   a jog step and returns to IDLE, the next call sees joy.base still 'L'/'R'
   and immediately triggers the next step — natural hold-to-repeat.
   ========================================================================= */
static void handle_joystick(void)
{
    if (!s_joy_mode) return;          /* base system mode — joystick disabled */

    JoyState_t joy = Joystick_GetState();
    if (!joy.connected) return;

    /* Emergency — enter FAULT so pilot lamp fires and reset is required */
    if (joy.emergency == 'P' || joy.base == 'X') {
        if (g_robot.fsm != STATE_FAULT) {
            MotorCtrl_Stop();
            g_robot.fsm              = STATE_FAULT;
            g_robot.comms.fault_code = 0x10u; /* joystick e-stop */
            s_run_mode               = RUN_IDLE;
            set_task(0x0000);
        }
        return;
    }

    /* All other commands only accepted when idle and gripper is not running */
    if (g_robot.fsm != STATE_IDLE) return;
    if (s_grip != GRIP_IDLE)       return;

    switch (joy.base) {
    case 'L':   /* jog CCW */
    case 'R': { /* jog CW  */
        float step = deg_to_rad((joy.base == 'R') ? JOY_JOG_STEP_DEG : -JOY_JOG_STEP_DEG);
        MotorCtrl_SetTarget(MotorCtrl_GetPosition_rad() + step);
        set_task(0x0008);
        s_move_start_ms = HAL_GetTick();
        g_robot.fsm     = STATE_RUNNING;
        s_run_mode      = RUN_JOG;
        break;
    }
    case 'A':                        /* Pick: down → close → up */
        gripper_start(true);
        break;
    case 'B':                        /* Place: down → open → up */
        gripper_start(false);
        break;
    case 'U':                        /* Manual gripper up       */
        Gripper_SetVertical(true);
        break;
    case 'D':                        /* Manual gripper down     */
        Gripper_SetVertical(false);
        break;
    case 'Y':                        /* Home                    */
        g_robot.fsm = STATE_HOMING;
        s_hom       = HOM_INIT;
        set_task(0x0001);
        Joystick_SendAudio('h');     /* @h = homing started     */
        break;
    default:
        break;
    }
}

/* =========================================================================
   Main FSM
   ========================================================================= */
static void fsm_run(void)
{
    /* E-stop overrides everything */
    if (g_robot.sensors.estop && g_robot.fsm != STATE_FAULT) {
        g_robot.fsm = STATE_FAULT;
        g_robot.comms.fault_code = 0x01u;
        MotorCtrl_Stop();
        set_task(0x0000);
    }

    uint16_t mode_reg = ModbusBridge_GetReg(0x01);

    switch (g_robot.fsm) {

    case STATE_INIT:
        g_robot.fsm = STATE_IDLE;
        break;

    case STATE_IDLE:
        set_task(0x0000);
        if (mode_reg & 0x01u) {                         /* Home             */
            ModbusBridge_SetReg(0x01, 0);
            g_robot.fsm = STATE_HOMING;
            s_hom       = HOM_INIT;
            set_task(0x0001);
        } else if ((mode_reg & 0x02u) || (ModbusBridge_GetReg(0x05) != 0u)) { /* Jog */
            int16_t step_raw = (int16_t)ModbusBridge_GetReg(0x05);
            if (step_raw != 0) {
                ModbusBridge_SetReg(0x01, 0);
                ModbusBridge_SetReg(0x05, 0); /* consume step so repeat writes don't chain */
                float cur = MotorCtrl_GetPosition_rad();
                MotorCtrl_SetTarget(cur + deg_to_rad((float)step_raw));
                set_task(0x0008); /* GoPoint */
                s_move_start_ms = HAL_GetTick();
                g_robot.fsm = STATE_RUNNING;
                s_run_mode  = RUN_JOG;
            }
        } else if (mode_reg & 0x04u) {                  /* Auto             */
            uint8_t pairs = (uint8_t)ModbusBridge_GetReg(0x22);
            if (pairs > 0u) {
                ModbusBridge_SetReg(0x01, 0); /* only consume mode when sequence is ready */
                s_seq_pairs  = (pairs > 8u) ? 8u : pairs;
                s_gripper_en = (ModbusBridge_GetReg(0x04) & 0x01u) != 0u;
                for (uint8_t i = 0; i < 16u; i++) {
                    s_seq_slots[i] = (int16_t)ModbusBridge_GetReg(0x12u + i);
                }
                s_seq_step          = 0;
                s_gripper_triggered = false;
                g_robot.fsm = STATE_RUNNING;
                s_run_mode  = RUN_AUTO;
            }
        } else if (mode_reg & 0x08u) {                  /* SetHome          */
            ModbusBridge_SetReg(0x01, 0);
            s_user_home_rad = MotorCtrl_GetPosition_rad();
        } else if (mode_reg & 0x10u) {                  /* Test — reserved  */
            ModbusBridge_SetReg(0x01, 0);
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
                g_robot.fsm = STATE_RUNNING;
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
            g_robot.fsm = STATE_IDLE;
            s_run_mode  = RUN_IDLE;
            break;
        }
        switch (s_run_mode) {
        case RUN_JOG:
        case RUN_POINT:
            if (MotorCtrl_IsAtTarget()) {
                g_robot.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            } else if ((HAL_GetTick() - s_move_start_ms) >= MOVE_TIMEOUT_MS) {
                MotorCtrl_Stop();
                g_robot.fsm = STATE_IDLE;
                s_run_mode  = RUN_IDLE;
                set_task(0x0000);
            }
            break;
        case RUN_AUTO:
            auto_run();
            break;
        default:
            g_robot.fsm = STATE_IDLE;
            break;
        }
        break;

    case STATE_FAULT:
        /* Latch — clear only if E-stop released AND reset button pressed    */
        if (!g_robot.sensors.estop && HwIo_GetResetBtn()) {
            g_robot.fsm             = STATE_IDLE;
            g_robot.comms.fault_code = 0u;
        }
        break;
    }
}

/* =========================================================================
   Public entry points
   ========================================================================= */

void App_Init(void)
{
    HwIo_Init();
    UartDma_Init();
    Joystick_Init();
    MotorCtrl_Init();
    ModbusBridge_Init();
    Motor_Enable();
    HAL_TIM_Base_Start_IT(&htim6);
    g_robot.fsm         = STATE_INIT;
    s_hom               = HOM_IDLE;
    s_run_mode          = RUN_IDLE;
    s_grip              = GRIP_IDLE;
    s_gripper_triggered = false;

    /* Initialise mode relay to match physical switch at power-on */
    s_joy_mode = HwIo_GetSelectedMode();
    Relay_SetSysmode(s_joy_mode);
}

void App_Run(void)
{
    /* Poll sensors into g_robot (non-ISR sensors) */
    /* E-stop: NC contact on PA5 — polarity inverted in hw_io.c (HIGH=active) */
    g_robot.sensors.estop         = HwIo_GetEStop();
    g_robot.sensors.selected_mode = HwIo_GetSelectedMode();
    g_robot.sensors.reset_btn     = HwIo_GetResetBtn();
    g_robot.sensors.proximity     = HwIo_GetProximity();

    /* Mode switch: ignore during FAULT (emergency can cause GPIO transients).
       Soft-stop any running motion so new mode takes control from clean state. */
    if (g_robot.sensors.selected_mode != s_joy_mode && g_robot.fsm != STATE_FAULT) {
        s_joy_mode = g_robot.sensors.selected_mode;
        Relay_SetSysmode(s_joy_mode);
        HwIo_ResetEstopDebounce();  /* discard relay-induced spike on PA5 */
        Joystick_SendAudio(s_joy_mode ? 'J' : 'S');
        if (g_robot.fsm == STATE_RUNNING) {
            MotorCtrl_Stop();
            g_robot.fsm = STATE_IDLE;
            s_run_mode  = RUN_IDLE;
            set_task(0x0000);
        }
    }

    /* Update debug mirror — expand g_robot in Live Expressions to see all */
    {
        JoyState_t _j = Joystick_GetState();
        g_robot.dbg.run_mode = (uint8_t)s_run_mode;
        g_robot.dbg.grip     = (uint8_t)s_grip;
        g_robot.dbg.joy_mode = (uint8_t)s_joy_mode;
        g_robot.dbg.joy_btn  = _j.base;
        g_robot.dbg.joy_conn = (uint8_t)_j.connected;
        g_robot.dbg.pos_deg  = rad_to_deg(MotorCtrl_GetPosition_rad());
        g_robot.dbg.vel_dps  = g_robot.motion.velocity_rps * 360.0f;
    }

    /* Drain UART RX → Modbus callbacks → register updates */
    UartDma_Process();

    /* Refresh Modbus read registers */
    ModbusBridge_Tick();

    /* Advance gripper sequence (runs independently of FSM) */
    gripper_seq_run();

    /* Handle joystick before FSM so commands take effect this iteration */
    handle_joystick();

    /* Run main FSM */
    fsm_run();

    /* Emergency pilot lamp — ON whenever robot is in FAULT */
    Relay_SetStatus(g_robot.fsm == STATE_FAULT);

    /* Audio feedback on FSM state transitions */
    {
        static FsmState_t s_prev_fsm = STATE_INIT;
        if (g_robot.fsm != s_prev_fsm) {
            switch (g_robot.fsm) {
            case STATE_HOMING: Joystick_SendAudio('h'); break; /* homing started */
            case STATE_FAULT:  Joystick_SendAudio('E'); break; /* fault          */
            case STATE_IDLE:
                if (s_prev_fsm == STATE_FAULT) Joystick_SendAudio('C'); /* cleared */
                break;
            default: break;
            }
            s_prev_fsm = g_robot.fsm;
        }
    }

    /* Periodic FSM state telemetry every 2 s for debug.
       Format: $ST,<FsmState_t>,<RunMode_t>  (0=INIT,1=HOMING,2=IDLE,3=RUNNING,4=FAULT) */
    uint32_t now = HAL_GetTick();
    if (now - s_tel_tick >= 2000u) {
        s_tel_tick = now;
        snprintf(s_dbg, sizeof(s_dbg), "$ST,%d,%d,%d\r\n",
                 (int)g_robot.fsm, (int)s_run_mode,
                 (int)g_robot.comms.fault_code);
        UartDma_SendTelemetry(s_dbg);
    }
}

#include "app_main.h"
#include "modbus_bridge.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
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
        /* Sequence complete */
        g_robot.fsm = STATE_IDLE;
        s_run_mode  = RUN_IDLE;
        set_task(0x0000);
        return;
    }

    if (s_seq_step > 0u && !MotorCtrl_IsAtTarget()) return; /* wait for previous move */

    uint8_t pair  = s_seq_step / 2u;
    bool    pick  = (s_seq_step & 1u) == 0u;
    int16_t raw   = s_seq_slots[pair * 2u + (pick ? 0u : 1u)];
    float   target = resolve_target(raw, 1u); /* sequence always uses index  */

    set_task(pick ? 0x0002u : 0x0004u); /* GoPick / GoPlace                */
    MotorCtrl_SetTarget(target);

    if (s_gripper_en && !pick) {
        /* Execute gripper place sequence when arriving at place position    */
        Gripper_SetClaw(false); /* close — place */
    }

    s_seq_step++;
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
        } else if (mode_reg & 0x02u) {                  /* Jog              */
            int16_t step_raw = (int16_t)ModbusBridge_GetReg(0x05);
            if (step_raw != 0) {
                ModbusBridge_SetReg(0x01, 0); /* only consume mode when step is ready */
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
                s_seq_step  = 0;
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
    MotorCtrl_Init();
    ModbusBridge_Init();
    Motor_Enable();
    HAL_TIM_Base_Start_IT(&htim6);
    g_robot.fsm = STATE_INIT;
    s_hom       = HOM_IDLE;
    s_run_mode  = RUN_IDLE;
}

void App_Run(void)
{
    /* Poll sensors into g_robot (non-ISR sensors) */
    /* E-stop disabled: NC switch grounds PA5 when unpressed — fix wiring
       before re-enabling. Wire NO contact to PA5 (HIGH=safe, LOW=stop).  */
    g_robot.sensors.estop         = false;
    g_robot.sensors.selected_mode = HwIo_GetSelectedMode();
    g_robot.sensors.reset_btn     = HwIo_GetResetBtn();
    g_robot.sensors.proximity     = HwIo_GetProximity();

    /* Drain UART RX → Modbus callbacks → register updates */
    UartDma_Process();

    /* Refresh Modbus read registers */
    ModbusBridge_Tick();

    /* Run main FSM */
    fsm_run();

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

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
     SWEEP      — triangular sweep with growing amplitude until sensor fires
     FIND_EDGE_B — continue same direction at creep until sensor turns OFF
     GO_CENTER  — creep toward (EdgeA + EdgeB) / 2
     SETTLE     — hold still for HOMING_SETTLE_MS then zero
   ========================================================================= */
typedef enum {
    HOM_IDLE = 0,
    HOM_INIT,
    HOM_SWEEP,        /* growing-amplitude oscillation searching for sensor  */
    HOM_FIND_EDGE_B,  /* sensor ON → keep going to find OFF (edge B)         */
    HOM_GO_CENTER,    /* creep to (EdgeA + EdgeB) / 2                        */
    HOM_SETTLE,       /* wait for arm to stop, then zero                     */
} HomingState_t;

static HomingState_t s_hom;
static float    s_sweep_start_rad;  /* position at start of current half-cycle */
static int8_t   s_sweep_dir;        /* +1 = forward, -1 = reverse              */
static float    s_sweep_amp_rad;    /* current half-cycle amplitude             */
static float    s_edge_a;           /* position where proximity turned ON       */
static float    s_edge_b;           /* position where proximity turned OFF      */
static bool     s_prox_prev;
static uint32_t s_settle_t;
static float    s_home_offset_rad;
static int8_t   s_go_center_dir;    /* tracks last dir in HOM_GO_CENTER — avoids repeated HomingCreep calls */

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
    bool     prox = HwIo_GetProximity();
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
        s_prox_prev       = prox;

        if (prox) {
            /* Already inside zone — creep forward to catch the OFF edge   */
            MotorCtrl_HomingCreep(+1);
            s_edge_a = pos;             /* treat current pos as edge A     */
            s_hom    = HOM_FIND_EDGE_B;
        } else {
            MotorCtrl_HomingCreep(s_sweep_dir);
            s_hom = HOM_SWEEP;
        }
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
                    g_robot.comms.fault_code = 1u; /* sensor not found     */
                    MotorCtrl_Stop();
                    break;
                }
                MotorCtrl_HomingCreep(s_sweep_dir);
            }

            /* Rising edge — proximity just turned ON → edge A found       */
            if (!s_prox_prev && prox) {
                s_edge_a = pos;
                /* Keep same direction at same slow speed to find edge B   */
                s_hom = HOM_FIND_EDGE_B;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_FIND_EDGE_B:
        /* Continue creeping in same direction until proximity turns OFF   */
        if (s_prox_prev && !prox) {
            s_edge_b = pos;
            MotorCtrl_Stop();
            s_go_center_dir = 0;    /* force first HomingCreep call in GO_CENTER */
            s_hom = HOM_GO_CENTER;
        }
        break;

    /* ------------------------------------------------------------------ */
    case HOM_GO_CENTER: {
        float center = (s_edge_a + s_edge_b) * 0.5f;
        float err    = center - pos;

        if (fabsf(err) <= deg_to_rad(0.5f)) {
            MotorCtrl_Stop();
            s_settle_t = now;
            s_hom      = HOM_SETTLE;
        } else {
            /* Only call HomingCreep when direction changes — prevents
               per-iteration calls from toggling velocity and shaking.   */
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
            MotorCtrl_Zero(s_home_offset_rad);
            MotorCtrl_SetTarget(s_home_offset_rad);
        }
        s_hom       = HOM_IDLE;
        g_robot.fsm = STATE_IDLE;
        set_task(0x0000);
        break;

    case HOM_IDLE:
    default:
        break;
    }

    s_prox_prev = prox;
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

    if (!MotorCtrl_IsAtTarget()) return; /* wait to arrive                  */

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
            ModbusBridge_SetReg(0x01, 0);
            int16_t step_raw = (int16_t)ModbusBridge_GetReg(0x05);
            if (step_raw != 0) {
                float cur = MotorCtrl_GetPosition_rad();
                MotorCtrl_SetTarget(cur + deg_to_rad((float)step_raw));
                set_task(0x0008); /* GoPoint */
                g_robot.fsm = STATE_RUNNING;
                s_run_mode  = RUN_JOG;
            }
        } else if (mode_reg & 0x04u) {                  /* Auto             */
            ModbusBridge_SetReg(0x01, 0);
            s_seq_pairs = (uint8_t)ModbusBridge_GetReg(0x22);
            if (s_seq_pairs > 8u) s_seq_pairs = 8u;   /* cap to available register range */
            s_gripper_en = (ModbusBridge_GetReg(0x04) & 0x01u) != 0u;
            for (uint8_t i = 0; i < 16u; i++) {
                s_seq_slots[i] = (int16_t)ModbusBridge_GetReg(0x12u + i);
            }
            s_seq_step  = 0;
            g_robot.fsm = STATE_RUNNING;
            s_run_mode  = RUN_AUTO;
        } else if (mode_reg & 0x08u) {                  /* SetHome          */
            ModbusBridge_SetReg(0x01, 0);
            int16_t off_raw = (int16_t)ModbusBridge_GetReg(MODBUS_REG_HOME_OFFSET);
            MotorCtrl_Zero(deg_to_rad((float)off_raw));
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
    g_robot.sensors.selected_mode = HwIo_GetSelectedMode();
    g_robot.sensors.reset_btn     = HwIo_GetResetBtn();
    g_robot.sensors.proximity     = HwIo_GetProximity();

    /* Drain UART RX → Modbus callbacks → register updates */
    UartDma_Process();

    /* Refresh Modbus read registers */
    ModbusBridge_Tick();

    /* Run main FSM */
    fsm_run();
}

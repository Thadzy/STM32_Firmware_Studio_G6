#include "test_config.h"
#if (ACTIVE_TEST == TEST_PHASE1)

#include "test_phase1.h"
#include "hw_io.h"
#include "params.h"
#include "system_state.h"
#include "main.h"   /* HAL_GetTick */

/* TIM3 owned by motor_controller (Phase 3) — borrowed here for encoder test */
extern TIM_HandleTypeDef htim3;

/* =========================================================================
   Phase 1 test — hw_io + encoder

   Live Expressions: add  g_robot  and expand sub-structs.

   Encoder: manually rotate the motor shaft and watch
            g_robot.motion.position_counts change.
            CW = positive, CCW = negative (swap if reversed on your hardware).
   ========================================================================= */

void TestPhase1_Init(void)
{
    HwIo_Init();
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void TestPhase1_Run(void)
{
    static uint32_t t_poll     = 0;
    static uint32_t t_vertical = 0;
    static uint32_t t_claw     = 0;
    static uint32_t t_motor    = 0;

    /* --- Encoder: track 16-bit overflow with a 32-bit accumulator --- */
    static int32_t  enc_accum  = 0;
    static uint16_t enc_last   = 0;
    uint16_t enc_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc_accum += (int16_t)(enc_now - enc_last);   /* signed 16-bit delta handles wrap */
    enc_last   = enc_now;
    g_robot.motion.position_counts = enc_accum;

    uint32_t now = HAL_GetTick();

    /* 100 Hz debounce poll */
    if (now - t_poll >= 10u) {
        t_poll = now;
        HwIo_Poll100Hz();

        g_robot.sensors.estop         = HwIo_GetEStop();
        g_robot.sensors.reed_up       = HwIo_GetReedSwitch(REED_UP);
        g_robot.sensors.reed_down     = HwIo_GetReedSwitch(REED_DOWN);
        g_robot.sensors.reed_open     = HwIo_GetReedSwitch(REED_OPEN);
        g_robot.sensors.reed_close    = HwIo_GetReedSwitch(REED_CLOSE);
        g_robot.sensors.proximity     = HwIo_GetProximity();
        g_robot.sensors.selected_mode = HwIo_GetSelectedMode();
        g_robot.sensors.reset_btn     = HwIo_GetResetBtn();
        g_robot.sensors.raw_adc      = HwIo_GetRawADC();
        g_robot.sensors.v_zero       = HwIo_GetVzero();
        g_robot.sensors.current_amps = HwIo_GetCurrentAmps();
    }

    /* Vertical relay toggles every 3 s */
    if (now - t_vertical >= 3000u) {
        t_vertical = now;
        g_robot.outputs.gripper_up = !g_robot.outputs.gripper_up;
        Gripper_SetVertical(g_robot.outputs.gripper_up);
    }

    /* Claw relay toggles every 4 s (offset from vertical so you can tell them apart) */
    if (now - t_claw >= 4000u) {
        t_claw = now;
        g_robot.outputs.claw_closed = !g_robot.outputs.claw_closed;
        Gripper_SetClaw(!g_robot.outputs.claw_closed);  /* SetClaw(open): open = NOT closed */
    }

    /* Motor test mode — change MOTOR_TEST_HOLD to 1 to lock PWM for current-sensor test,
       or leave at 0 for the normal ramp.                                               */
#define MOTOR_TEST_HOLD  0

#if MOTOR_TEST_HOLD
    /* Holds motor at MOTOR_VOLT_LIMIT_PWM continuously — stall it and watch raw_adc_avg */
    Motor_Enable();
    Motor_SetPWM((int16_t)MOTOR_VOLT_LIMIT_PWM);
    g_robot.motion.motor_pwm      = (int16_t)MOTOR_VOLT_LIMIT_PWM;
    g_robot.outputs.motor_enabled = true;
#else
    /* Normal ramp every 2 s within 6 V safe limit */
    if (now - t_motor >= 2000u) {
        t_motor = now;
        static uint8_t step = 0;
        static const int16_t seq[] = {
             0,
             (int16_t)(MOTOR_VOLT_LIMIT_PWM / 2u),
             (int16_t) MOTOR_VOLT_LIMIT_PWM,
             (int16_t)(MOTOR_VOLT_LIMIT_PWM / 2u),
             0,
            -(int16_t)(MOTOR_VOLT_LIMIT_PWM / 2u),
            -(int16_t) MOTOR_VOLT_LIMIT_PWM,
            -(int16_t)(MOTOR_VOLT_LIMIT_PWM / 2u)
        };
        g_robot.motion.motor_pwm = seq[step % 8u];
        g_robot.outputs.motor_enabled = true;
        step++;
        Motor_Enable();
        Motor_SetPWM(g_robot.motion.motor_pwm);
    }
#endif
}

#endif /* ACTIVE_TEST == TEST_PHASE1 */

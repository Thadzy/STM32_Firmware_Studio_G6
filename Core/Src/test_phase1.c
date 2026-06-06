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

   Live Expressions: add  RobotState  and expand sub-structs.

   Encoder: manually rotate the motor shaft and watch
            RobotState.motion.position_counts change.
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
    RobotState.motion.position_counts = enc_accum;

    uint32_t now = HAL_GetTick();

    /* 100 Hz debounce poll */
    if (now - t_poll >= 10u) {
        t_poll = now;
        HwIo_Poll100Hz();

        RobotState.sensors.estop         = HwIo_GetEStop();
        RobotState.sensors.reed_up       = HwIo_GetReedSwitch(REED_UP);
        RobotState.sensors.reed_down     = HwIo_GetReedSwitch(REED_DOWN);
        RobotState.sensors.reed_open     = HwIo_GetReedSwitch(REED_OPEN);
        RobotState.sensors.reed_close    = HwIo_GetReedSwitch(REED_CLOSE);
        RobotState.sensors.proximity     = HwIo_GetProximity();
        RobotState.sensors.selected_mode = HwIo_GetSelectedMode();
        RobotState.sensors.reset_btn     = HwIo_GetResetBtn();
        RobotState.sensors.raw_adc      = HwIo_GetRawADC();
        RobotState.sensors.v_zero       = HwIo_GetVzero();
        RobotState.sensors.current_amps = HwIo_GetCurrentAmps();
    }

    /* Vertical relay toggles every 3 s */
    if (now - t_vertical >= 3000u) {
        t_vertical = now;
        RobotState.outputs.gripper_up = !RobotState.outputs.gripper_up;
        Gripper_SetVertical(RobotState.outputs.gripper_up);
    }

    /* Claw relay toggles every 4 s (offset from vertical so you can tell them apart) */
    if (now - t_claw >= 4000u) {
        t_claw = now;
        RobotState.outputs.claw_closed = !RobotState.outputs.claw_closed;
        Gripper_SetClaw(!RobotState.outputs.claw_closed);  /* SetClaw(open): open = NOT closed */
    }

    /* Motor test mode — change MOTOR_TEST_HOLD to 1 to lock PWM for current-sensor test,
       or leave at 0 for the normal ramp.                                               */
#define MOTOR_TEST_HOLD  0

#if MOTOR_TEST_HOLD
    /* Holds motor at MOTOR_VOLT_LIMIT_PWM continuously — stall it and watch raw_adc_avg */
    Motor_Enable();
    Motor_SetPWM((int16_t)MOTOR_VOLT_LIMIT_PWM);
    RobotState.motion.motor_pwm      = (int16_t)MOTOR_VOLT_LIMIT_PWM;
    RobotState.outputs.motor_enabled = true;
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
        RobotState.motion.motor_pwm = seq[step % 8u];
        RobotState.outputs.motor_enabled = true;
        step++;
        Motor_Enable();
        Motor_SetPWM(RobotState.motion.motor_pwm);
    }
#endif
}

#endif /* ACTIVE_TEST == TEST_PHASE1 */

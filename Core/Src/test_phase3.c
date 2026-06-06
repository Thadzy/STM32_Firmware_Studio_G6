#include "test_config.h"
#if (ACTIVE_TEST == TEST_PHASE3)

#include "test_phase3.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim6;

/* =========================================================================
   Phase 3 test — Kalman filter + motor controller

   BEFORE FLASHING
   ---------------
   1. Position the arm at a safe middle-of-range location.
   2. Ensure no cable/mechanical obstruction within ±90° of start position.
   3. MOTOR_VOLT_LIMIT_PWM is not enforced by the controller — the full PID
      output (up to ±50 PWM) will be applied. If motion seems violent, reduce
      SCURVE_VMAX_RADS or SCURVE_AMAX_RADS2 in params.h and re-flash.

   WHAT HAPPENS
   ------------
   The arm steps between 0° and +90° (1.5708 rad) every 5 s.
   Telemetry is sent every 100 ms on LPUART1 (230400 8E1).

   Live Expressions — add RobotState and expand:
     RobotState.motion.position_counts  encoder count (raw)
     RobotState.motion.velocity_rps     Kalman velocity (rev/s)
     RobotState.motion.accel_rps2       Kalman acceleration (rev/s²)
     RobotState.motion.motor_pwm        PWM command sent to motor
     RobotState.sensors.estop           E-stop state (triggers motor off)

   Terminal output (230400 8E1):
     $MC3,pos_deg,vel_rads,acc_rads2,pwm\r\n

   REQUIRED stm32g4xx_it.c ADDITIONS  (already applied in this build)
   -------------------------------------------------------------------
   USER CODE BEGIN Includes:
     #include "motor_controller.h"
     #include "hw_io.h"
     #include "params.h"

   USER CODE BEGIN 1 (bottom of file):
     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
         if (htim->Instance != TIM6) return;
         MotorCtrl_Tick1kHz();
         static uint16_t s_div = 0;
         if (++s_div >= OUTER_LOOP_DIVIDER) {
             s_div = 0;
             HwIo_Poll100Hz();
             MotorCtrl_Tick100Hz();
         }
     }
   ========================================================================= */

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

void TestPhase3_Init(void)
{
    HwIo_Init();
    UartDma_Init();
    MotorCtrl_Init();
    Motor_Enable();   /* energise motor-power relay (PB12) before TIM6 runs */

    /* Start TIM6 interrupt — fires MotorCtrl_Tick1kHz every 1 ms           */
    HAL_TIM_Base_Start_IT(&htim6);

    /* Command: hold at home (0 rad) on startup                              */
    MotorCtrl_SetTarget(0.0f);
}

void TestPhase3_Run(void)
{
    static uint32_t t_step  = 0;
    static uint32_t t_telem = 0;
    static bool     at_90   = false;

    uint32_t now = HAL_GetTick();

    /* --- Step command: toggle 0° ↔ +90° every 5 s ----------------------- */
    if (now - t_step >= 5000u) {
        t_step = now;
        at_90  = !at_90;
        MotorCtrl_SetTarget(at_90 ? (M_PI / 2.0f) : 0.0f);
    }

    /* --- Telemetry every 100 ms ------------------------------------------ */
    if (now - t_telem >= 100u) {
        t_telem = now;

        float pos_deg = MotorCtrl_GetPosition_rad() * (180.0f / M_PI);
        float vel     = RobotState.motion.velocity_rps * (2.0f * M_PI); /* rad/s */
        float acc     = RobotState.motion.accel_rps2  * (2.0f * M_PI);
        int16_t pwm   = RobotState.motion.motor_pwm;

        char buf[64];
        snprintf(buf, sizeof(buf), "$MC3,%.2f,%.3f,%.2f,%d\r\n",
                 (double)pos_deg, (double)vel, (double)acc, (int)pwm);
        UartDma_SendTelemetry(buf);
    }

    UartDma_Process();
}

#endif /* ACTIVE_TEST == TEST_PHASE3 */

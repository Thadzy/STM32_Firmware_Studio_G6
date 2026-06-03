#ifndef __PARAMS_H
#define __PARAMS_H

#include <stdint.h>

/* --- Timing ----------------------------------------------------------------*/
#define CONTROL_LOOP_HZ         1000u
#define OUTER_LOOP_HZ           100u
#define OUTER_LOOP_DIVIDER      (CONTROL_LOOP_HZ / OUTER_LOOP_HZ)  /* = 10 */

/* --- Debounce --------------------------------------------------------------*/
/* E-Stop: 8 × 10 ms = 80 ms consecutive LOW reads required to trigger.
   Clears immediately on a single HIGH read (fail-safe). */
#define ESTOP_DEBOUNCE_THRESHOLD    8u

/* --- ADC / Current Sensor (WCS1800, ±35 A) ---------------------------------*/
/* Zero is auto-calibrated at boot (64-sample average while motor is off).
   CS_DIV_RATIO: voltage divider between sensor output and PA0.
                 Set to 1.0 if wired directly; read from schematic to confirm.
   CS_SENSITIVITY: WCS1800 datasheet value in V/A. At 5 V supply: ~0.040 V/A.
                   To calibrate: apply a known current I_known, then:
                   CS_SENSITIVITY = (v_sensor_at_I - v_zero) / I_known         */
#define CS_VREF             3.3f
#define CS_ADC_COUNTS       4096.0f
#define CS_DIV_RATIO        1.0f        /* TODO: verify from schematic          */
#define CS_SENSITIVITY      0.040f      /* V/A — WCS1800 @ 5 V typical          */
#define CS_EMA_ALPHA        0.1f        /* EMA weight: smaller = smoother       */

/* --- Motor PWM (TIM1: Prescaler=169, ARR=50 → fPWM ≈ 19.6 kHz) -----------*/
#define MOTOR_PWM_MAX           50u     /* hardware ceiling = ARR = 100 % duty = 24 V */
#define MOTOR_SUPPLY_VOLTS      24.0f
#define MOTOR_SAFE_VOLTS        6.0f
/* Software duty cap: (6/24) × 50 = 12  — raise gradually once motion is verified */
#define MOTOR_VOLT_LIMIT_PWM    12u

/* --- Encoder (ATM103, 2048 CPR → TIM3 TI12 quadrature × 4 = 8192 cnt/rev) */
/* Workspace: 360° rotational, cable hard-limit 540°.
   Direct drive assumed — if a gearbox is fitted, multiply ENCODER_CPR by the ratio.
   MAX_POSITION_COUNTS = 1 full revolution = 8192 counts.
   The 540° cable limit is 12288 counts — firmware must never command past MAX. */
#define ENCODER_CPR             8192u
#define WORKSPACE_MAX_DEG       360u
#define CABLE_MAX_DEG           540u        /* absolute hardware limit — never approach */
#define MAX_POSITION_COUNTS     8192        /* = ENCODER_CPR × (WORKSPACE_MAX_DEG / 360) */

/* --- PID Gains (seed values — tune on hardware) ----------------------------*/
#define PID_SPEED_KP            0.5f
#define PID_SPEED_KI            0.1f
#define PID_SPEED_KD            0.0f

#define PID_POS_KP              2.0f
#define PID_POS_KI              0.0f
#define PID_POS_KD              0.05f

/* --- Kalman Filter ---------------------------------------------------------*/
#define KALMAN_DT               0.001f  /* 1 ms — matches TIM6 period */

/* --- UART / Modbus ---------------------------------------------------------*/
#define UART_TX_BUF_SIZE        1024u
#define UART_TX_HIGH_WATERMARK  800u
/* Modbus T3.5 inter-frame guard: must be ≥ 1.75 ms at 230400 baud */
#define MODBUS_T35_DELAY_MS     2u

/* --- Homing ----------------------------------------------------------------*/
#define HOMING_SPEED_PWM        6       /* slow creep ≈ 2.9 V, well under MOTOR_VOLT_LIMIT_PWM */

#endif /* __PARAMS_H */

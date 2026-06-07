#ifndef __PARAMS_H
#define __PARAMS_H

#include <stdint.h>

/* --- Timing ----------------------------------------------------------------*/
#define CONTROL_LOOP_HZ         1000u
#define OUTER_LOOP_HZ           100u
#define OUTER_LOOP_DIVIDER      (CONTROL_LOOP_HZ / OUTER_LOOP_HZ)  /* = 10 */

/* --- Debounce --------------------------------------------------------------*/
/* E-Stop: 3 × 10 ms = 30 ms consecutive LOW reads → immediate motor stop.
   Then ESTOP_VERIFY_MS elapses with motor off (EMI gone); if pin is still
   LOW → real press → latch FAULT.  If pin went HIGH → EMI noise → resume.
   Clears immediately on a single HIGH read (fail-safe). */
#define ESTOP_DEBOUNCE_THRESHOLD    3u
#define ESTOP_VERIFY_MS             50u   /* ms to wait with motor off before confirming fault */

/* Mode selector (PA6): a maintained switch, not momentary — a long hold is
   fine and rejects motor-PWM-switching EMI bursts coupled onto the line.
   100 × 10 ms = 1000 ms of stable reading required before the mode flips.    */
#define MODE_SWITCH_DEBOUNCE_TICKS  100u

/* Reset button (PA7): momentary switch.
   20 × 10 ms = 200 ms of stable reading required to reject motor-PWM EMI.    */
#define RESET_BTN_DEBOUNCE_TICKS    20u

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
/* Software duty cap: motion verified — raised to 50 % duty (12 V).
   Previous 25 % (6 V) was too low; motor was creeping at ~1 deg/s during homing. */
#define MOTOR_VOLT_LIMIT_PWM    25u

/* --- Encoder (ATM103, 2048 CPR → TIM3 TI12 quadrature × 4 = 8192 cnt/rev)
   ENCODER_DIRECTION: set +1 if positive PWM → increasing counts.
                      set -1 if positive PWM → decreasing counts (flip here). */
#define ENCODER_DIRECTION       (-1)    /* +1 = positive PWM → increasing counts.
                                           -1 = positive PWM → decreasing counts (flip if motor runs away). */
/* Workspace: 360° rotational, cable hard-limit 540°.
   Direct drive assumed — if a gearbox is fitted, multiply ENCODER_CPR by the ratio.
   MAX_POSITION_COUNTS = 1 full revolution = 8192 counts.
   The 540° cable limit is 12288 counts — firmware must never command past MAX. */
#define ENCODER_CPR             8192u
#define WORKSPACE_MAX_DEG       360u
#define CABLE_MAX_DEG           540u        /* absolute hardware limit — never approach */
#define MAX_POSITION_COUNTS     8192        /* = ENCODER_CPR × (WORKSPACE_MAX_DEG / 360) */
#define CABLE_MAX_COUNTS        12288       /* = ENCODER_CPR × (CABLE_MAX_DEG / 360)    */

/* --- PID Gains — Velocity loop (inner, 1 kHz) ------------------------------*/
#define PID_SPEED_KP            1.58f
#define PID_SPEED_KI            0.2f
#define PID_SPEED_KD            0.0f
#define PID_SPEED_IMAX          40.0f   /* integral clamp (PWM units)         */

/* --- PID Gains — Position loop (outer, 100 Hz) -----------------------------*/
#define PID_POS_KP              3.0f
#define PID_POS_KI              0.01f
#define PID_POS_KD              0.0f
#define PID_POS_IMAX            100.0f  /* integral clamp (rad)               */

/* --- Feedforward Gains -----------------------------------------------------*/
/* FF_VELOCITY : direct velocity FF to inner loop.  Units: PWM / (rad/s)
   FF_ACCEL    : acceleration FF to inner loop.     Units: PWM / (rad/s²)     */
#define FF_VELOCITY             3.03f
#define FF_ACCEL                0.1f
#define FF_DISTURBANCE          4.0f

/* --- S-Curve Trajectory Limits ---------------------------------------------*/
#define SCURVE_VMAX_RADS        7.304f      /* rad/s   — maximum velocity     */
#define SCURVE_AMAX_RADS2       27.49f      /* rad/s²  — maximum acceleration */
#define SCURVE_JMAX_RADS3       1400.0f     /* rad/s³  — maximum jerk         */

/* --- ZVD Input Shaper (Zero Vibration Derivative) -------------------------*/
/* Hardware identified: fn = 12.13 Hz, ζ = 0.041
   ωd = ωn × √(1−ζ²) = 76.17 rad/s  →  Td = π/ωd = 41.2 ms
   At 100 Hz outer loop: T2 = 4 steps, T3 = 8 steps
   K = exp(−ζπ/√(1−ζ²)) = 0.8790,  Σ = (1+K)² = 3.531                      */
#define ZVD_NATURAL_FREQ_HZ     12.13f
#define ZVD_DAMPING_RATIO       0.041f
#define ZVD_A1                  0.2832f     /* impulse amplitude at t=0       */
#define ZVD_A2                  0.4981f     /* impulse amplitude at t=T2      */
#define ZVD_A3                  0.2189f     /* impulse amplitude at t=T3      */
#define ZVD_T2_STEPS            4u          /* delay in 100 Hz steps          */
#define ZVD_T3_STEPS            8u
#define ZVD_BUF_SIZE            9u          /* ZVD_T3_STEPS + 1               */

/* --- Kalman Filter ---------------------------------------------------------*/
#define KALMAN_DT               0.001f      /* 1 ms — matches TIM6 period     */
#define KALMAN_Q_POS            1e-5f       /* process noise — position       */
#define KALMAN_Q_VEL            1e-3f       /* process noise — velocity       */
#define KALMAN_Q_ACC            0.1f        /* process noise — acceleration   */
#define KALMAN_Q_JERK           1.0f        /* process noise — jerk           */
#define KALMAN_R_POS            2e-6f       /* measurement noise — position   */

/* --- Position settling window ----------------------------------------------*/
#define POSITION_DEADBAND_RAD   0.0175f     /* ≈ 1° — IsAtTarget threshold    */
#define VELOCITY_SETTLED_RADS   0.05f       /* rad/s — near-stop threshold    */

/* --- Software Safety Stack -------------------------------------------------*/

/* --- Heartbeat -------------------------------------------------------------*/
#define HEARTBEAT_TIMEOUT_MS    2000u   /* soft-stop to IDLE if PC silent this long */

/* --- UART / Modbus ---------------------------------------------------------*/
#define UART_TX_BUF_SIZE        1024u
#define UART_TX_HIGH_WATERMARK  800u
/* Modbus T3.5 inter-frame guard: must be ≥ 1.75 ms at 230400 baud */
#define MODBUS_T35_DELAY_MS     2u

/* --- Homing ----------------------------------------------------------------*/
#define HOMING_SPEED_PWM        6       /* legacy PWM reference — not used by controller      */
/* Reduced from 0.8 → 0.4 rad/s (~23 deg/s).
   At 0.8 rad/s with the 8-tick (80 ms) debounce window, the arm travels
   ≈3.7° before the edge is confirmed — enough to exit a narrow sensor zone
   before the latch fires.  At 0.4 rad/s travel during debounce is ≈1.8°,
   well within any practical proximity sensor detection window.
   Raise back toward 0.8 once sensor wiring is confirmed and edge is seen. */
#define HOMING_VEL_RADS         0.8f    /* creep velocity for edge search (rad/s) ~46 deg/s   */
/* --- Two-stage homing ------------------------------------------------------*/
#define HOMING_FAST_VEL_RADS    2.0f    /* Stage-1 coarse search (~114 deg/s)                 */
#define HOMING_PREC_VEL_RADS    0.5f    /* Stage-3 precision search (~29 deg/s). Above motor
                                           dead zone → smooth, no stick-slip crawl. Repeatability
                                           comes from A/B latency cancelling at EQUAL velocity,
                                           not from low speed — keep fast enough to move cleanly. */
#define HOMING_BACKOFF_DEG      5.0f    /* extra travel past prox-OFF to clear the window     */
#define HOMING_BACKOFF_MAX_DEG  60.0f   /* abort if sensor never clears during backoff        */
#define HOMING_ACCEL_RADS2      4.0f    /* velocity ramp rate during homing (rad/s²)          */
#define HOMING_WIGGLE_STEP_DEG  90.0f   /* amplitude increment per wiggle step (degrees)      */
/* Hardware-calibrated offset: proximity sensor center → working 0°.
   = position reading (in degrees) when arm is at physical home, taken from
     s_pos_counts × (360.0 / 8192.0) in Live Expressions after homing.
   Negative  = physical home is in the −encoder direction from sensor center.
   To shift working 0° further by N°, subtract N from this value.
   Example: measured −3.12°, want extra +10° working offset → set −13.12°.
   Adjust reg 0x32 (int16 degrees) for small per-session fine-tuning.          */
#define HOME_OFFSET_DEG         (-4.658f)   /* degrees from sensor center to physical home */
#define HOMING_WIGGLE_MAX_DEG   180.0f  /* max search range from start before FAULT            */
#define HOMING_OVERSHOOT_DEG         5.0f   /* min deg past edge A before even checking for clear  */
#define HOMING_OVERSHOOT_MAX_DEG    45.0f   /* abort if sensor never clears within this travel     */
#define HOMING_MAX_SEARCH_DEG        40.0f  /* max deg to travel in FIND_EDGE_B before fault       */
#define HOMING_CENTER_TIMEOUT_MS     30000u /* ms cap on GO_CENTER — proceeds to SETTLE regardless */
#define HOMING_SETTLE_MS        1000u   /* ms to wait after reaching center before zeroing    */
#define MOVE_TIMEOUT_MS         10000u  /* ms — jog/P2P abort to IDLE if IsAtTarget not met   */

/* --- Gripper Sequence ------------------------------------------------------*/
#define GRIP_TIMEOUT_MS         3000u   /* per-step timeout if reed switch missing */

/* --- Joystick (ESP32 Bluetooth gamepad via USART3 115200 8N1) -------------*/
#define JOY_JOG_STEP_DEG        10.0f   /* degrees per L/R step (Option 2 only)   */

/* Option 1 — Velocity Bypass: joystick directly commands inner-loop velocity.
   Tune JOY_JOG_VEL_RADS so the arm feels responsive but controllable.
   At 1.0 rad/s ≈ 57°/s the arm traverses 360° in ~6 s.                      */
#define JOY_JOG_VEL_RADS        1.0f    /* rad/s — direct velocity during L/R hold */

/* PC jog step velocity: velocity-bypass drive toward the discrete step target.
   Uses the same inner-loop bypass as homing/joystick (no position PID).
   1.0 rad/s ≈ 57°/s — fast enough to feel crisp, slow enough to stop cleanly
   within the 1° deadband without overshoot.                                   */
#define JOG_PC_VEL_RADS         1.0f    /* rad/s — PC discrete jog step velocity  */
#define JOG_PC_ACCEL_RADS2      5.0f    /* rad/s² — velocity ramp rate for jog    */

/* Option 2 — S-curve discrete jog steps use the normal SCURVE_ limits.
   No Amax/Jmax override — smooth S-curve shape at normal speed.
   ZVD is bypassed during jog (see motor_controller.c JogStepEngage).         */

/* --- Pick & Place ---------------------------------------------------------*/
#define P2P_INDEX_COUNT         72u     /* 72 slots × 5° = 360° workspace                     */
/* Home offset Modbus register (writable by PC, outside base-system range)   */
#define MODBUS_REG_HOME_OFFSET  0x32u

/* --- Auto-Tune Relay Feedback (Åström-Hägglund) ---------------------------*/
/* Firmware holds still this many ms before activating the relay so the arm
   damps any oscillations from the preceding move.
   Must be <= RelayAnalyzer.settle_time_s × 1000 on the PC side.             */
#define AT_SETTLE_MS            1000u

#endif /* __PARAMS_H */

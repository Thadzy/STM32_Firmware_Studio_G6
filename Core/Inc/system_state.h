#ifndef __SYSTEM_STATE_H
#define __SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

/* FSM states — used by app_main.c */
typedef enum {
    STATE_INIT    = 0,
    STATE_HOMING,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_FAULT
} FsmState_t;

/* Central system state — one global instance: g_robot.
   Fields marked volatile are written from the TIM6 ISR (1 kHz / 100 Hz).
   All other fields are written from the main loop only.

   In STM32CubeIDE Live Expressions, add:  g_robot
   then expand the sub-structs to watch each group live. */
typedef struct {

    FsmState_t fsm;             /* current FSM state                        */

    /* --- Sensors (written by HwIo_Poll100Hz — called from TIM6 ISR) ------ */
    struct {
        volatile bool     estop;           /* latches true after 80 ms LOW   */
        volatile bool     reed_up;
        volatile bool     reed_down;
        volatile bool     reed_open;
        volatile bool     reed_close;
        volatile bool     proximity;
        volatile uint16_t raw_adc;         /* last raw ADC count             */
        volatile float    v_zero;          /* auto-calibrated zero voltage   */
        volatile float    current_amps;
                 bool     selected_mode;   /* read on-demand, not from ISR   */
                 bool     reset_btn;       /* read on-demand, not from ISR   */
    } sensors;

    /* --- Motion (written by motor_controller from TIM6 ISR 1 kHz) -------- */
    struct {
        volatile int32_t position_counts;   /* encoder counts from home      */
        volatile float   velocity_rps;      /* revolutions per second        */
        volatile float   accel_rps2;        /* rev / s²                      */
        volatile int16_t motor_pwm;         /* current PWM command (-50…+50) */
    } motion;

    /* --- Outputs (written by app_main / gripper commands) ----------------- */
    struct {
        bool gripper_up;    /* true = arm UP                                 */
        bool claw_closed;   /* true = claw CLOSED (gripping)                 */
        bool motor_enabled; /* true = motor-power relay energised            */
    } outputs;

    /* --- Comms (written by uart_dma_manager / modbus_bridge / app_main) -- */
    struct {
        uint16_t heartbeat;         /* mirrors Modbus reg 0x00               */
        uint8_t  fault_code;        /* reason for STATE_FAULT                */
        uint16_t tx_used;           /* TX ring buffer bytes in use           */
        bool     t35_active;        /* true during Modbus inter-frame gap    */
        uint32_t telemetry_drops;   /* cumulative dropped telemetry strings  */
        uint16_t last_rx_len;       /* last LPUART1 packet length received   */
    } comms;

    /* --- Debug mirror (updated every App_Run — expand in Live Expressions) -
       In STM32CubeIDE Live Expressions, add ONE entry:  g_robot
       then expand each sub-struct to watch everything live.

       dbg.run_mode : 0=IDLE  1=JOG  2=AUTO  3=POINT  4=TEST
       dbg.grip     : 0=IDLE  1=PICK_DOWN  2=PICK_CLOSE  3=PICK_UP
                      4=PLACE_DOWN  5=PLACE_OPEN  6=PLACE_UP
       dbg.joy_mode : 0=base_system  1=joystick
       dbg.joy_btn  : ASCII  'O'=idle  'L'=left  'R'=right  'A' 'B' 'Y' 'U' 'D'
       dbg.joy_conn : 0=gamepad_not_paired  1=gamepad_connected
       dbg.pos_deg  : motor position in degrees (easier to read than position_counts)
       dbg.vel_dps  : velocity in degrees/s
    -------------------------------------------------------------------------- */
    struct {
        uint8_t  run_mode;
        uint8_t  grip;
        uint8_t  joy_mode;
        char     joy_btn;
        uint8_t  joy_conn;
        float    pos_deg;
        float    vel_dps;
    } dbg;

} RobotState_t;

extern RobotState_t g_robot;

#endif /* __SYSTEM_STATE_H */

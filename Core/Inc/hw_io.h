#ifndef __HW_IO_H
#define __HW_IO_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    REED_UP = 0,
    REED_DOWN,
    REED_OPEN,
    REED_CLOSE,
    REED_COUNT
} ReedSwitch_t;

/* Call once after MX peripheral init, before starting TIM6 */
void  HwIo_Init(void);

/* Call from the TIM6 ISR every 10 ticks (100 Hz outer loop tick) */
void  HwIo_Poll100Hz(void);

/* --- Digital inputs --------------------------------------------------------*/
bool  HwIo_GetEStop(void);                  /* true = E-Stop active (debounced) */
bool  HwIo_GetReedSwitch(ReedSwitch_t sw);  /* true = magnet detected (debounced) */
bool  HwIo_GetProximity(void);              /* true = object detected (debounced) */
bool  HwIo_GetResetBtn(void);               /* true = button pressed (debounced) */
bool  HwIo_GetSelectedMode(void);           /* true = mode switch active */

/* --- Current sense ---------------------------------------------------------*/
uint16_t HwIo_GetRawADC(void);      /* last raw ADC count — watch during stall to verify sensor */
float    HwIo_GetVzero(void);       /* auto-calibrated zero voltage — should be stable after init */
float    HwIo_GetCurrentAmps(void); /* EMA-filtered, auto-zeroed current in Amperes */

/* --- Actuators -------------------------------------------------------------*/
void  Gripper_SetVertical(bool go_up);      /* drives up/down solenoid relays */
void  Gripper_SetClaw(bool open);           /* drives claw solenoid relay */

/* pwm: -MOTOR_PWM_MAX … +MOTOR_PWM_MAX (sign = direction) */
void  Motor_SetPWM(int16_t pwm);
void  Motor_Enable(void);                   /* energises motor-power relay */
void  Motor_Disable(void);                  /* zeroes PWM then drops motor-power relay */

void  Relay_SetStatus(bool on);             /* system-status indicator relay */

#endif /* __HW_IO_H */

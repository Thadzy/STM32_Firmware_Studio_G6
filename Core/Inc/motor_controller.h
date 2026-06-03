#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include <stdbool.h>

/* Motor controller — cascade PID with feedforward, S-curve trajectory,
   ZVD input shaper, and 4-state Kalman filter.

   Call order (from stm32g4xx_it.c TIM6 ISR):
     every tick (1 kHz) : MotorCtrl_Tick1kHz()
     every 10th tick    : MotorCtrl_Tick100Hz()                               */

void  MotorCtrl_Init(void);
void  MotorCtrl_Tick1kHz(void);
void  MotorCtrl_Tick100Hz(void);
void  MotorCtrl_SetTarget(float target_rad);
void  MotorCtrl_Stop(void);
bool  MotorCtrl_IsAtTarget(void);
float MotorCtrl_GetPosition_rad(void);

#endif /* __MOTOR_CONTROLLER_H */

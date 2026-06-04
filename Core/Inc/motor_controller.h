#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include <stdint.h>
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
/* Homing helpers */
void  MotorCtrl_HomingCreep(int8_t dir);      /* constant-vel creep: +1=fwd, -1=rev */
void  MotorCtrl_Zero(float home_offset_rad);  /* redefine current pos as home+offset */
void  MotorCtrl_SyncTrajectory(void);         /* re-seed S-curve from current Kalman pos before SetTarget */
bool  MotorCtrl_IsAtPosition(void);           /* position within deadband, no velocity gate — for homing */

#endif /* __MOTOR_CONTROLLER_H */

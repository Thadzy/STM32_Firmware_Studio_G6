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

/* Auto-tune: apply synthesised PID gains at run-time.
   loop: AT_LOOP_VELOCITY (0) → inner speed PID
         AT_LOOP_POSITION (1) → outer position PID
   Resets the corresponding integrator to avoid bump.
   Uses a __disable_irq() critical section — call from main-loop only.       */
void  MotorCtrl_SetPidGains(uint8_t loop, float kp, float ki, float kd);

/* ---- Joystick jog helpers --------------------------------------------------
   Two mutually exclusive options — use one set or the other, never both.

   OPTION 1 — Velocity Bypass (preferred for smooth continuous jogging):
     MotorCtrl_JogVelocity()  call every App_Run while stick is held.
       Bypasses S-curve, ZVD, and the outer position PID entirely.
       Feeds vel_rads directly into the inner velocity PID.
       Position integral is zeroed each outer tick to prevent windup.
     MotorCtrl_JogRelease()   call when stick is released.
       Locks onto current Kalman position.  Re-seeds S-curve with the
       actual velocity so the position loop re-engages without a bump
       (bumpless transfer: trajectory initial conditions match plant state).
     MotorCtrl_IsJogActive()  true while velocity bypass is running.

   OPTION 2 — Aggressive Step Mode (if outer-loop bypass is not acceptable):
     MotorCtrl_JogStepEngage()    call on first stick press.
       Bypasses ZVD (pass-through) and raises Amax / Jmax so each
       discrete step (JOY_JOG_STEP_DEG) completes before the next
       App_Run, eliminating ZVD buffer clash.
     MotorCtrl_JogStepDisengage() call when stick is released.
       Restores default S-curve limits and re-enables ZVD.
       Clears position integral (accumulated during aggressive stepping). */

/* ZVD bypass — call before HAL_TIM_Base_Start_IT to take effect from first tick */
void  MotorCtrl_SetZvdBypass(bool bypass);

/* Option 1 */
void  MotorCtrl_JogVelocity(float vel_rads); /* ISR-safe: volatile write     */
void  MotorCtrl_JogRelease(void);            /* call from main-loop only      */
bool  MotorCtrl_IsJogActive(void);

/* Option 2 */
void  MotorCtrl_JogStepEngage(void);         /* call from main-loop only      */
void  MotorCtrl_JogStepDisengage(void);      /* call from main-loop only      */

#endif /* __MOTOR_CONTROLLER_H */

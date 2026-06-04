#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include <stdbool.h>

/* Parsed state of the last frame received from the ESP32 gamepad.
   ESP32 sends "[base][emergency][status]\r\n" on every state change.

   base:      O=idle  L=left  R=right  U=up  D=down
              A=btn_A  B=btn_B  Y=btn_Y  M=btn_M  F=ry_fwd  X=emergency
   emergency: O=none  P=pressed
   connected: true when a Bluetooth controller is paired to the ESP32       */
typedef struct {
    char base;
    char emergency;
    bool connected;
} JoyState_t;

/* Register USART3 DMA callback — call once from App_Init()                  */
void       Joystick_Init(void);

/* Return the most recent parsed state (safe to call from main loop)         */
JoyState_t Joystick_GetState(void);

/* Send a single-character audio command to the ESP32 buzzer.
   Commands: 'h'=homing started  'H'=homing done  'E'=fault  'C'=fault cleared
   Transmitted as two bytes: '@' + cmd                                       */
void       Joystick_SendAudio(char cmd);

#endif /* __JOYSTICK_H */

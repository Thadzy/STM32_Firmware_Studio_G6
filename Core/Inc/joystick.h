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
    bool connected;
    char base;
    char emergency;
} JoyState_t;

/* Initialize USART3 DMA reception */
void Joystick_Init(void);

/* Get the most recent gamepad state */
JoyState_t Joystick_GetState(void);


#endif /* __JOYSTICK_H */

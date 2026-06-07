#include "joystick.h"
#include "uart_dma_manager.h"
#include "main.h"
#include <stdint.h>

/* Latest parsed gamepad state — written by ISR callback, read by main loop  */
static JoyState_t s_state = { false, 'O', 'O' };
static volatile uint32_t s_joy_last_tick = 0;

/* ── RX callback (called from USART3 DMA idle-line ISR) ──────────────────── */
static void joy_rx_cb(const uint8_t *buf, uint16_t size)
{
    /* Frame format: "[base][emergency][status]\r\n"
       Scan forward until we find a valid triplet so partial or multi-frame
       buffers are handled without dropping valid data.                       */
    for (uint16_t i = 0; i + 2 < size; i++) {
        char b = (char)buf[i];
        char e = (char)buf[i + 1];
        char s = (char)buf[i + 2];

        if ((s == 'N' || s == 'C') && (e == 'O' || e == 'P')) {
            /* Cortex-M4 single-word writes are atomic; struct fits in 3 bytes.
               No critical section needed for this read/write pattern.       */
            s_state.base      = b;
            s_state.emergency = e;
            s_state.connected = (s == 'C');
            s_joy_last_tick   = HAL_GetTick();

            /* Handshake: echo action char back to ESP32 to trigger buzzer confirm beep */
            if (b != 'O' || e != 'O') {
                uint8_t echo = (uint8_t)b;
                (void)UartDma_SendJoystick(&echo, 1u);
            }
            return; /* take the first valid frame, ignore the rest           */
        }
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void Joystick_Init(void)
{
    UartDma_SetUsart3RxCb(joy_rx_cb);
}

JoyState_t Joystick_GetState(void)
{
    return s_state;
}

void Joystick_SendAudio(char cmd)
{
    uint8_t msg[2] = { '@', (uint8_t)cmd };
    UartDma_SendJoystick(msg, 2);
}

bool Joystick_IsAlive(void)
{
    /* If we haven't seen a valid packet in 500ms, consider the ESP32 disconnected */
    if (s_joy_last_tick == 0) return true; /* Hasn't connected yet */
    return (HAL_GetTick() - s_joy_last_tick) < 500u;
}

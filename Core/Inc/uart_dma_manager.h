#ifndef __UART_DMA_MANAGER_H
#define __UART_DMA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/* Callback fired when an IDLE-terminated packet arrives */
typedef void (*UartRxCb_t)(const uint8_t *data, uint16_t len);

/* Call once after MX peripheral init, before starting TIM6 */
void     UartDma_Init(void);

/* Register RX callbacks (called from HAL RxEvent ISR context) */
void     UartDma_SetLpuartRxCb(UartRxCb_t cb);
void     UartDma_SetUsart3RxCb(UartRxCb_t cb);

/* TX — LPUART1 (TDM: Modbus + Telemetry on same wire)
   SendModbus    : always queued if ring buffer has space
   SendTelemetry : dropped if high-watermark exceeded OR T3.5 guard active */
bool     UartDma_SendModbus(const uint8_t *data, uint16_t len);
bool     UartDma_SendTelemetry(const char *str);

/* TX — USART3 (joystick/ESP32) */
bool     UartDma_SendJoystick(const uint8_t *data, uint16_t len);

/* Call from main loop (and later from 1 kHz TIM6 tick) to re-kick TX after
   T3.5 guard expires — safe to call from any context */
void     UartDma_Process(void);

/* Status */
uint16_t UartDma_GetTxUsed(void);       /* bytes currently queued in TX ring */
bool     UartDma_IsT35Active(void);     /* true while Modbus inter-frame gap enforced */

#endif /* __UART_DMA_MANAGER_H */

#include "uart_dma_manager.h"
#include "main.h"
#include "params.h"
#include <string.h>

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart3;

/* =========================================================================
   RX — both UARTs use ReceiveToIdle_DMA.
   HAL fires HAL_UARTEx_RxEventCallback on IDLE line; we re-arm immediately.
   Half-transfer interrupt is disabled to avoid spurious mid-packet callbacks.
   ========================================================================= */

#define LPUART_RX_BUF   256u
#define USART3_RX_BUF   128u

static uint8_t    s_lp_rx[LPUART_RX_BUF];
static uint8_t    s_u3_rx[USART3_RX_BUF];
static UartRxCb_t s_lp_rx_cb = NULL;
static UartRxCb_t s_u3_rx_cb = NULL;

/* =========================================================================
   TX ring buffer — LPUART1 only (Modbus + Telemetry share one buffer).

   Layout: standard circular byte buffer.
   Message queue: parallel descriptor queue tracks (length, is_modbus) per
   queued message.  Used by the TxCplt callback to know when a Modbus frame
   finishes, so T3.5 can be armed precisely after the last byte leaves.
   ========================================================================= */

static uint8_t  s_tx[UART_TX_BUF_SIZE];
static uint16_t s_tx_head = 0;          /* next write position              */
static uint16_t s_tx_tail = 0;          /* next DMA read position           */
static bool     s_tx_busy = false;      /* DMA transfer in progress         */
static uint16_t s_tx_dma_len = 0;       /* length of active DMA transfer    */

#define TX_MSG_MAX  16u
typedef struct { uint16_t len; bool is_modbus; } TxMsg_t;
static TxMsg_t  s_msgs[TX_MSG_MAX];
static uint8_t  s_msg_wr = 0;
static uint8_t  s_msg_rd = 0;
static uint8_t  s_modbus_queued = 0;    /* Modbus frames still in ring buf  */

/* T3.5 inter-frame guard */
static bool     s_t35_active     = false;
static uint32_t s_t35_start_tick = 0;

/* ---- USART3 TX (simple: one frame at a time, no ring buffer needed) ---- */
#define USART3_TX_BUF  128u
static uint8_t s_u3_tx[USART3_TX_BUF];
static bool    s_u3_busy = false;

/* =========================================================================
   Private helpers
   ========================================================================= */

static inline uint8_t msg_count(void)
{
    return (uint8_t)((s_msg_wr - s_msg_rd) % TX_MSG_MAX);
}

static inline uint16_t tx_used(void)
{
    return (uint16_t)((s_tx_head - s_tx_tail + UART_TX_BUF_SIZE) % UART_TX_BUF_SIZE);
}

static inline uint16_t tx_free(void)
{
    return (uint16_t)(UART_TX_BUF_SIZE - 1u - tx_used());
}

static bool tx_write(const uint8_t *data, uint16_t len)
{
    if (len > tx_free() || msg_count() >= (TX_MSG_MAX - 1u)) return false;

    uint16_t to_end = UART_TX_BUF_SIZE - s_tx_head;
    if (len <= to_end) {
        memcpy(&s_tx[s_tx_head], data, len);
    } else {
        memcpy(&s_tx[s_tx_head], data, to_end);
        memcpy(&s_tx[0], data + to_end, len - to_end);
    }
    s_tx_head = (s_tx_head + len) % UART_TX_BUF_SIZE;
    return true;
}

static void tx_kick(void)
{
    if (s_tx_busy) return;
    if (s_tx_head == s_tx_tail) return;

    /* T3.5 guard: hold telemetry until inter-frame gap has elapsed */
    if (s_t35_active) {
        if ((HAL_GetTick() - s_t35_start_tick) < MODBUS_T35_DELAY_MS) {
            if (msg_count() > 0 && !s_msgs[s_msg_rd].is_modbus) {
                return;  /* defer telemetry — let the next Process() call retry */
            }
        } else {
            s_t35_active = false;
        }
    }

    /* Send contiguous chunk: either up to head, or to end of buffer (then
       the next TxCplt callback will kick the wrapped remainder). */
    s_tx_dma_len = (s_tx_tail < s_tx_head)
                       ? (uint16_t)(s_tx_head - s_tx_tail)
                       : (uint16_t)(UART_TX_BUF_SIZE - s_tx_tail);

    s_tx_busy = true;
    HAL_UART_Transmit_DMA(&hlpuart1, &s_tx[s_tx_tail], s_tx_dma_len);
}

/* =========================================================================
   Public API
   ========================================================================= */

void UartDma_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_lp_rx, LPUART_RX_BUF);
    __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_u3_rx, USART3_RX_BUF);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void UartDma_SetLpuartRxCb(UartRxCb_t cb) { s_lp_rx_cb = cb; }
void UartDma_SetUsart3RxCb(UartRxCb_t cb) { s_u3_rx_cb = cb; }

bool UartDma_SendModbus(const uint8_t *data, uint16_t len)
{
    if (!tx_write(data, len)) return false;

    s_msgs[s_msg_wr] = (TxMsg_t){ .len = len, .is_modbus = true };
    s_msg_wr = (s_msg_wr + 1u) % TX_MSG_MAX;
    s_modbus_queued++;

    tx_kick();
    return true;
}

bool UartDma_SendTelemetry(const char *str)
{
    /* Drop: TX buffer too full (reserve space for Modbus replies) */
    if (tx_used() >= UART_TX_HIGH_WATERMARK) return false;

    /* Drop: Modbus frame still in ring buffer or T3.5 gap in effect */
    if (s_modbus_queued > 0) return false;
    if (s_t35_active && (HAL_GetTick() - s_t35_start_tick) < MODBUS_T35_DELAY_MS)
        return false;

    uint16_t len = (uint16_t)strlen(str);
    if (!tx_write((const uint8_t *)str, len)) return false;

    s_msgs[s_msg_wr] = (TxMsg_t){ .len = len, .is_modbus = false };
    s_msg_wr = (s_msg_wr + 1u) % TX_MSG_MAX;

    tx_kick();
    return true;
}

bool UartDma_SendJoystick(const uint8_t *data, uint16_t len)
{
    if (s_u3_busy || len > USART3_TX_BUF) return false;
    memcpy(s_u3_tx, data, len);
    s_u3_busy = true;
    HAL_UART_Transmit_DMA(&huart3, s_u3_tx, len);
    return true;
}

void UartDma_Process(void)
{
    tx_kick();  /* re-check after T3.5 expiry */
}

uint16_t UartDma_GetTxUsed(void)   { return tx_used(); }
bool     UartDma_IsT35Active(void) { return s_t35_active && (HAL_GetTick() - s_t35_start_tick) < MODBUS_T35_DELAY_MS; }

/* =========================================================================
   HAL callbacks (override weak symbols — called from UART/DMA ISR context)
   ========================================================================= */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &hlpuart1) {
        if (s_lp_rx_cb && Size > 0u) s_lp_rx_cb(s_lp_rx, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_lp_rx, LPUART_RX_BUF);
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
    } else if (huart == &huart3) {
        if (s_u3_rx_cb && Size > 0u) s_u3_rx_cb(s_u3_rx, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_u3_rx, USART3_RX_BUF);
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1) {
        /* Advance tail */
        s_tx_tail = (s_tx_tail + s_tx_dma_len) % UART_TX_BUF_SIZE;
        s_tx_busy = false;

        /* Dequeue all messages whose bytes were covered by this DMA transfer */
        uint16_t remaining = s_tx_dma_len;
        while (remaining > 0u && msg_count() > 0u) {
            TxMsg_t *m = &s_msgs[s_msg_rd];
            if (remaining >= m->len) {
                remaining -= m->len;
                if (m->is_modbus) {
                    s_modbus_queued--;
                    s_t35_active     = true;
                    s_t35_start_tick = HAL_GetTick();
                }
                s_msg_rd = (s_msg_rd + 1u) % TX_MSG_MAX;
            } else {
                m->len -= remaining;  /* message spans next DMA transfer */
                remaining = 0u;
            }
        }

        tx_kick();  /* send next chunk or next message immediately */
    } else if (huart == &huart3) {
        s_u3_busy = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Re-arm RX on any UART error to prevent the DMA from stalling */
    if (huart == &hlpuart1) {
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_lp_rx, LPUART_RX_BUF);
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
    } else if (huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_u3_rx, USART3_RX_BUF);
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

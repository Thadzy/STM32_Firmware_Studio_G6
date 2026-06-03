#include "test_config.h"
#if (ACTIVE_TEST == TEST_PHASE2)

#include "test_phase2.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
   Phase 2 test — uart_dma_manager: TX ring buffer, RX idle DMA, T3.5

   Live Expressions: add  g_robot  and expand g_robot.comms

   What to observe
   ---------------
   g_robot.comms.tx_used        : rises as telemetry is queued, falls as DMA drains it
   g_robot.comms.t35_active     : briefly true after each fake Modbus reply is sent
   g_robot.comms.telemetry_drops: increments when telemetry is blocked (T3.5 or watermark)
   g_robot.comms.last_rx_len    : length of last packet received on LPUART1

   Hardware test
   -------------
   Connect ST-Link VCP (LPUART1) to a PC terminal (230400 8E1).
   You should see lines like:  $TELEM,00042,0.00,0.00,0.00\r\n
   Every 5 s a fake 8-byte Modbus reply is sent — watch for the 1-2 ms gap
   before the next telemetry string (t35_active goes true then clears).
   Send any bytes from the terminal to test RX (last_rx_len will update).
   ========================================================================= */

/* Fake Modbus reply: FC=03, 4 regs, addr=21 (0x15) — not a real reply, just
   for testing the T3.5 guard.  CRC is valid so serial_bridge.py accepts it. */
static const uint8_t k_fake_modbus[] = {
    0x15, 0x03, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xF6, 0x5A   /* CRC-16 of the above */
};

static void on_lpuart_rx(const uint8_t *data, uint16_t len)
{
    g_robot.comms.last_rx_len = len;
    /* Phase 4 (modbus_bridge) will process the payload — nothing else needed here */
}

void TestPhase2_Init(void)
{
    HwIo_Init();
    UartDma_Init();
    UartDma_SetLpuartRxCb(on_lpuart_rx);
}

void TestPhase2_Run(void)
{
    static uint32_t t_telem  = 0;
    static uint32_t t_modbus = 0;
    static uint32_t counter  = 0;

    uint32_t now = HAL_GetTick();

    /* Send telemetry every 100 ms */
    if (now - t_telem >= 100u) {
        t_telem = now;

        char buf[64];
        snprintf(buf, sizeof(buf), "$TELEM,%05lu,%.2f,%.2f,%.2f\r\n",
                 counter++,
                 (double)g_robot.motion.velocity_rps,
                 (double)g_robot.motion.accel_rps2,
                 (double)g_robot.sensors.current_amps);

        if (!UartDma_SendTelemetry(buf)) {
            g_robot.comms.telemetry_drops++;
        }
    }

    /* Send a fake Modbus reply every 5 s to exercise T3.5 guard */
    if (now - t_modbus >= 5000u) {
        t_modbus = now;
        UartDma_SendModbus(k_fake_modbus, sizeof(k_fake_modbus));
    }

    /* Update g_robot.comms so Live Expressions reflects current state */
    g_robot.comms.tx_used    = UartDma_GetTxUsed();
    g_robot.comms.t35_active = UartDma_IsT35Active();

    /* Keep TX moving (re-kicks after T3.5 expires) */
    UartDma_Process();
}

#endif /* ACTIVE_TEST == TEST_PHASE2 */

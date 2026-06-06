#include "canbus.h"
#include "hw_io.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

/* Global state — add g_can to Live Expressions */
CanState_t g_can;

/* ── Private helpers ────────────────────────────────────────────────────────── */

/* Encode DLC byte count → FDCAN DLC field value */
static uint32_t dlc_encode(uint8_t n)
{
    static const uint32_t tbl[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    if (n > 8) n = 8;
    return tbl[n];
}

static void CanBus_Transmit(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    FDCAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.Identifier          = can_id;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = dlc_encode(dlc);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data);
}

static void CanBus_Failsafe(void)
{
    Gripper_SetVertical(false);   /* solenoid de-energised → spring return */
    Gripper_SetClaw(false);       /* solenoid de-energised → spring open   */
    g_can.relay_state = 0x00u;
    g_can.node_state  = CAN_NODE_FAILSAFE;
}

/* Reset the watchdog timer — called on any valid master message */
static void CanBus_KickWatchdog(void)
{
    g_can.last_hb_tick   = HAL_GetTick();
    g_can.watchdog_armed = true;
}

/* ── Incoming message handlers ──────────────────────────────────────────────── */

static void Handle_MasterHeartbeat(uint8_t *data, uint8_t dlc)
{
    if (dlc < 1) return;
    CanBus_KickWatchdog();

    if (data[0] == CAN_MASTER_OPERATIONAL) {
        g_can.node_state = CAN_NODE_OPERATIONAL;
    } else if (data[0] == CAN_MASTER_STOPPED) {
        CanBus_Failsafe();
    }
}

static void Handle_WriteRelay(uint8_t *data, uint8_t dlc)
{
    if (dlc < 3) return;
    if (data[1] != CAN_TARGET_RELAY) return;
    if (g_can.node_state != CAN_NODE_OPERATIONAL) return;

    uint8_t mask = data[2];

    /* Apply each relay bit to the gripper hardware.
     * Bits 0 and 1 are mutually exclusive vertical directions; last write wins. */
    if (mask & RELAY_BIT_GRIPPER_UP)    Gripper_SetVertical(true);
    if (mask & RELAY_BIT_GRIPPER_DOWN)  Gripper_SetVertical(false);
    if (mask & RELAY_BIT_GRIPPER_CLOSE) Gripper_SetClaw(false);  /* close = claw not open */
    if (mask & RELAY_BIT_GRIPPER_OPEN)  Gripper_SetClaw(true);

    g_can.relay_state = mask;

    /* Send Write Ack 0x310: [0x11, 0x00, actual_state] */
    uint8_t resp[3] = { CAN_INSTR_WRITE_ACK, CAN_TARGET_RELAY, g_can.relay_state };
    CanBus_Transmit(CAN_ID_CMD_RESP, resp, 3);
}

static void Handle_ReadOpto(uint8_t *data, uint8_t dlc)
{
    if (dlc < 2) return;
    if (data[1] != CAN_TARGET_OPTO) return;

    /* Build opto bitmask from reed switches (sensor feedback, bits 4-6) */
    uint8_t opto = 0x00u;
    if (HwIo_GetReedSwitch(REED_UP))    opto |= OPTO_BIT_SENSOR_UP;
    if (HwIo_GetReedSwitch(REED_DOWN))  opto |= OPTO_BIT_SENSOR_DOWN;
    if (HwIo_GetReedSwitch(REED_OPEN) ||
        HwIo_GetReedSwitch(REED_CLOSE)) opto |= OPTO_BIT_SENSOR_OC;

    /* Send Read Response 0x310: [0x21, 0x10, opto_bitmask] */
    uint8_t resp[3] = { CAN_INSTR_READ_RESP, CAN_TARGET_OPTO, opto };
    CanBus_Transmit(CAN_ID_CMD_RESP, resp, 3);
}

static void Handle_CommandRequest(uint8_t *data, uint8_t dlc)
{
    /* Reset watchdog on any valid command — spec section 3.1 note */
    CanBus_KickWatchdog();

    if (dlc < 1) return;
    switch (data[0]) {
        case CAN_INSTR_WRITE_REQ: Handle_WriteRelay(data, dlc); break;
        case CAN_INSTR_READ_REQ:  Handle_ReadOpto(data, dlc);   break;
        default: break;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void CanBus_Init(void)
{
    /* Filter 0 — exact match 0x210 (Command Request from master to this node) */
    FDCAN_FilterTypeDef f;
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_DUAL;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = CAN_ID_CMD_REQ;   /* 0x210 */
    f.FilterID2    = CAN_ID_CMD_REQ;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);

    /* Filter 1 — exact match 0x600 (Master Heartbeat broadcast) */
    f.FilterIndex  = 1;
    f.FilterID1    = CAN_ID_MASTER_HB;  /* 0x600 */
    f.FilterID2    = CAN_ID_MASTER_HB;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);

    /* Reject all frames that don't match a filter */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* Enable the FDCAN1 interrupt line in NVIC (CubeMX .ioc does not enable it,
     * so do it here to keep the CAN driver self-contained). */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    HAL_FDCAN_Start(&hfdcan1);

    g_can.node_state     = CAN_NODE_BOOTING;
    g_can.watchdog_armed = false;
    g_can.relay_state    = 0x00u;
}

void CanBus_Tick(void)
{
    if (!g_can.watchdog_armed) return;
    if (g_can.node_state == CAN_NODE_FAILSAFE) return;

    if ((HAL_GetTick() - g_can.last_hb_tick) > CAN_WATCHDOG_TIMEOUT_MS) {
        CanBus_Failsafe();
    }
}

CanNodeState_t CanBus_GetNodeState(void)
{
    return g_can.node_state;
}

/* ── HAL RX callback — called from stm32g4xx_it.c USER CODE ────────────────── */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK) {
        uint8_t dlc = (uint8_t)(rx_hdr.DataLength >> 16);

        if (rx_hdr.Identifier == CAN_ID_MASTER_HB) {
            Handle_MasterHeartbeat(rx_data, dlc);
        } else if (rx_hdr.Identifier == CAN_ID_CMD_REQ) {
            Handle_CommandRequest(rx_data, dlc);
        }
    }
}

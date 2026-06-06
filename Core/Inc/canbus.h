#ifndef __CANBUS_H
#define __CANBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* CAN ID construction — spec section 2 */
#define MAKE_CAN_ID(fc, nid)  (((uint32_t)((fc) & 0x07u) << 8) | ((nid) & 0xFFu))

#define CAN_NODE_ID           0x10u

/* Receive IDs (node listens for these) */
#define CAN_ID_MASTER_HB      MAKE_CAN_ID(0x6, 0x00)        /* 0x600 — Master Heartbeat  */
#define CAN_ID_CMD_REQ        MAKE_CAN_ID(0x2, CAN_NODE_ID) /* 0x210 — Command Request   */

/* Transmit IDs (node sends these) */
#define CAN_ID_CMD_RESP       MAKE_CAN_ID(0x3, CAN_NODE_ID) /* 0x310 — Command Response  */

/* Instruction bytes — spec section 4.2 / 4.3 */
#define CAN_INSTR_WRITE_REQ   0x10u
#define CAN_INSTR_WRITE_ACK   0x11u
#define CAN_INSTR_READ_REQ    0x20u
#define CAN_INSTR_READ_RESP   0x21u

/* I/O target banks — spec section 4.1 */
#define CAN_TARGET_RELAY      0x00u
#define CAN_TARGET_OPTO       0x10u

/* Master state bytes — spec section 3.1 */
#define CAN_MASTER_OPERATIONAL 0x05u
#define CAN_MASTER_STOPPED     0x00u

/* Node states — spec section 3.2 */
typedef enum {
    CAN_NODE_BOOTING     = 0x00,
    CAN_NODE_OPERATIONAL = 0x05,
    CAN_NODE_FAILSAFE    = 0xFF,
} CanNodeState_t;

/* Relay bitmask — spec section 9 (Relay Bank 0 Map) */
#define RELAY_BIT_GRIPPER_UP    (1u << 0)
#define RELAY_BIT_GRIPPER_DOWN  (1u << 1)
#define RELAY_BIT_GRIPPER_CLOSE (1u << 2)
#define RELAY_BIT_GRIPPER_OPEN  (1u << 3)

/* Opto bitmask — spec section 9 (Opto Bank 0 Map, sensor feedback bits 4-6) */
#define OPTO_BIT_SENSOR_UP      (1u << 4)
#define OPTO_BIT_SENSOR_DOWN    (1u << 5)
#define OPTO_BIT_SENSOR_OC      (1u << 6)  /* open/close sensor */

/* Watchdog timeout — spec section 3: 2000 ms */
#define CAN_WATCHDOG_TIMEOUT_MS 2000u

/* All CAN bus state in one struct — add g_can to Live Expressions */
typedef struct {
    CanNodeState_t node_state;
    uint32_t       last_hb_tick;
    bool           watchdog_armed;
    uint8_t        relay_state;   /* last relay bitmask applied to gripper */
} CanState_t;

extern CanState_t g_can;

void           CanBus_Init(void);
void           CanBus_Tick(void);
CanNodeState_t CanBus_GetNodeState(void);

#endif /* __CANBUS_H */

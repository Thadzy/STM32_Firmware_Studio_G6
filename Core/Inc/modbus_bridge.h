#ifndef __MODBUS_BRIDGE_H
#define __MODBUS_BRIDGE_H

#include <stdint.h>

/* Modbus RTU bridge — slave address 21 (0x15).
   Supported function codes:
     FC03 0x03 — Read Holding Registers
     FC06 0x06 — Write Single Register
     FC16 0x10 — Write Multiple Registers

   Register range: 0x00–0x32 (see CLAUDE.md for full map).

   Call from main loop:
     ModbusBridge_Init()  — once at startup
     ModbusBridge_Tick()  — every App_Run iteration
                            refreshes read registers from g_robot state       */

void     ModbusBridge_Init(void);
void     ModbusBridge_Tick(void);           /* refresh read regs + heartbeat  */

uint16_t ModbusBridge_GetReg(uint8_t addr);
void     ModbusBridge_SetReg(uint8_t addr, uint16_t val);

#endif /* __MODBUS_BRIDGE_H */

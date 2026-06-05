#ifndef __MODBUS_BRIDGE_H
#define __MODBUS_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

/* Modbus RTU bridge — slave address 21 (0x15).
   Supported function codes:
     FC03 0x03 — Read Holding Registers
     FC06 0x06 — Write Single Register
     FC16 0x10 — Write Multiple Registers

   Register range: 0x00–0x3C (see CLAUDE.md for full map).

   Registers 0x00–0x31 : existing application registers
   Register  0x32       : home offset (existing, MODBUS_REG_HOME_OFFSET)
   Registers 0x33–0x3C : Auto-Tune relay feedback (new)

   Call from main loop:
     ModbusBridge_Init()  — once at startup
     ModbusBridge_Tick()  — every App_Run iteration;
                            refreshes read registers from g_robot state       */

/* ── Auto-Tune register addresses ─────────────────────────────────────────
   WRITE  (PC → STM32, FC06 / FC16)                                          */
#define REG_AT_CMD          0x33u  /* 0=Idle 1=StartRelay 2=ApplyGains 3=Abort */
#define REG_AT_RELAY_AMP    0x34u  /* relay amplitude d, int16 × 10            */
#define REG_AT_SETPOINT     0x35u  /* operating setpoint, int16 × 10, degrees  */
#define REG_AT_LOOP         0x36u  /* 0=velocity inner (1kHz) 1=pos outer (100Hz)*/
#define REG_AT_HYSTERESIS   0x37u  /* relay dead-band, int16 × 100, degrees    */
#define REG_AT_NEW_KP       0x38u  /* synthesised Kp, int16 × 1000             */
#define REG_AT_NEW_KI       0x39u  /* synthesised Ki, int16 × 1000             */
#define REG_AT_NEW_KD       0x3Au  /* synthesised Kd, int16 × 1000             */
/*  READ  (STM32 → PC, FC03)                                                  */
#define REG_AT_STATUS       0x3Bu  /* 0=Idle 1=Settling 2=Active 3=Done 4=Fault*/
#define REG_AT_CYCLES       0x3Cu  /* completed oscillation cycles (uint16)    */

/* ── AT_CMD values ─────────────────────────────────────────────────────── */
#define AT_CMD_IDLE         0u
#define AT_CMD_START_RELAY  1u
#define AT_CMD_APPLY_GAINS  2u
#define AT_CMD_ABORT        3u

/* ── AT_LOOP values ────────────────────────────────────────────────────── */
#define AT_LOOP_VELOCITY    0u   /* inner velocity PID (1 kHz)  */
#define AT_LOOP_POSITION    1u   /* outer position PID (100 Hz) */

/* ── AT_STATUS values — mirror of RelayState in relay_analyzer.py ──────── */
#define AT_STATUS_IDLE      0u
#define AT_STATUS_SETTLING  1u
#define AT_STATUS_ACTIVE    2u
#define AT_STATUS_DONE      3u
#define AT_STATUS_FAULT     4u

/* ── Shared auto-tune parameter struct ────────────────────────────────────
   Written by modbus_bridge (UART-ISR / main-loop context).
   Read by motor_controller (TIM6-ISR context).
   All float fields are 4-byte aligned → single-instruction read/write on
   Cortex-M4; no torn reads.
   `cmd` and `status` are volatile uint8_t for cross-context visibility.    */
typedef struct {
    volatile uint8_t  cmd;          /* AT_CMD_* — written from UART ISR      */
    uint8_t           loop_target;  /* AT_LOOP_*                              */
    float             amplitude;    /* relay d (rad/s for pos, PWM for vel)   */
    float             setpoint;     /* operating point, degrees               */
    float             hysteresis;   /* switching dead-band, degrees           */
    float             new_kp;       /* ZN-synthesised Kp                      */
    float             new_ki;       /* ZN-synthesised Ki                      */
    float             new_kd;       /* ZN-synthesised Kd                      */
    volatile uint8_t  status;       /* AT_STATUS_* — written from TIM6 ISR   */
    volatile uint16_t cycles;       /* oscillation count — written from ISR   */
} AutoTune_Params_t;

extern AutoTune_Params_t g_at;

/* ── Public API ──────────────────────────────────────────────────────────*/
void     ModbusBridge_Init(void);
void     ModbusBridge_Tick(void);    /* refresh read regs + heartbeat + AT    */

uint16_t ModbusBridge_GetReg(uint8_t addr);
void     ModbusBridge_SetReg(uint8_t addr, uint16_t val);

#endif /* __MODBUS_BRIDGE_H */

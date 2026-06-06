#include "modbus_bridge.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "system_state.h"
#include "hw_io.h"
#include "params.h"
#include "main.h"
#include "app_main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* -------------------------------------------------------------------------
   Constants
   ------------------------------------------------------------------------- */
#define MB_ADDR         21u      /* slave address                            */
#define MB_FC_READ      0x03u
#define MB_FC_WRITE1    0x06u
#define MB_FC_WRITEN    0x10u
#define MB_REG_COUNT    0x3Du    /* 0x00 … 0x3C inclusive (10 AT regs added) */
#define MB_MIN_FRAME    4u       /* addr + fc + CRC (minimum valid frame)    */

/* Heartbeat tokens (ASCII: "YA" = 22881, "HI" = 18537) */
#define HB_YA   22881u
#define HB_HI   18537u
#define HB_PERIOD_MS  200u      /* robot sends YA every 200 ms (≈5 Hz)      */

/* -------------------------------------------------------------------------
   Internal state
   ------------------------------------------------------------------------- */
static uint16_t s_regs[MB_REG_COUNT];  /* shadow register bank              */
static uint32_t s_hb_tick;             /* last heartbeat send time           */
static bool     s_hb_pending;          /* YA sent, waiting for HI reply      */

/* Auto-tune shared parameter block — extern declared in modbus_bridge.h    */
AutoTune_Params_t g_at = { 0 };
static void send_gains_telemetry(uint8_t loop);

/* -------------------------------------------------------------------------
   CRC-16/Modbus
   ------------------------------------------------------------------------- */
static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------
   Send exception response
   ------------------------------------------------------------------------- */
static void send_exception(uint8_t fc, uint8_t code)
{
    uint8_t frame[5];
    frame[0] = MB_ADDR;
    frame[1] = fc | 0x80u;
    frame[2] = code;
    uint16_t crc = crc16(frame, 3);
    frame[3] = (uint8_t)(crc & 0xFFu);
    frame[4] = (uint8_t)(crc >> 8);
    UartDma_SendModbus(frame, 5);
}

/* -------------------------------------------------------------------------
   FC03 — Read Holding Registers
   ------------------------------------------------------------------------- */
static void handle_fc03(const uint8_t *req)
{
    uint16_t start = ((uint16_t)req[2] << 8) | req[3];
    uint16_t count = ((uint16_t)req[4] << 8) | req[5];

    if (count == 0 || count > 125u || (start + count) > MB_REG_COUNT) {
        send_exception(MB_FC_READ, 0x02);
        return;
    }

    /* Response: [addr][fc][byte_count][data...][crc_lo][crc_hi] */
    uint8_t resp[3 + 250 + 2];
    resp[0] = MB_ADDR;
    resp[1] = MB_FC_READ;
    resp[2] = (uint8_t)(count * 2u);
    for (uint16_t i = 0; i < count; i++) {
        uint16_t v   = s_regs[start + i];
        resp[3 + i*2]     = (uint8_t)(v >> 8);
        resp[3 + i*2 + 1] = (uint8_t)(v & 0xFFu);
    }
    uint16_t len = 3u + count * 2u;
    uint16_t crc = crc16(resp, len);
    resp[len]     = (uint8_t)(crc & 0xFFu);
    resp[len + 1] = (uint8_t)(crc >> 8);
    UartDma_SendModbus(resp, len + 2u);
}

/* -------------------------------------------------------------------------
   Apply a single register write to system state
   ------------------------------------------------------------------------- */
static void apply_reg_write(uint8_t addr, uint16_t val)
{
    if (addr >= MB_REG_COUNT) return;
    s_regs[addr] = val;

    /* Interpret side effects */
    switch (addr) {
    case 0x00:
        /* Heartbeat reply from PC */
        if (val == HB_HI) {
            s_hb_pending         = false;
            RobotState.comms.heartbeat = HB_HI;
        }
        break;

    case 0x02:
        /* Manual gripper command */
        if (val & 0x01u) Gripper_SetVertical(false);  /* Down */
        else if (val == 0u) Gripper_SetVertical(true); /* Up  */
        if (val & 0x02u) Gripper_SetClaw(true);       /* Open  */
        if (val & 0x04u) Gripper_SetClaw(false);      /* Close */
        break;

    /* ---- Auto-tune WRITE registers 0x33–0x3A ----------------------------- */
    case REG_AT_CMD:
        /* Write cmd last so motor_controller never sees a partially-written
           parameter block.  The ISR latches all params atomically at START.  */
        g_at.cmd = (uint8_t)(val & 0xFFu);
        break;

    case REG_AT_RELAY_AMP:
        g_at.amplitude   = (float)(int16_t)val / 10.0f;
        break;

    case REG_AT_SETPOINT:
        g_at.setpoint    = (float)(int16_t)val / 10.0f;   /* degrees */
        break;

    case REG_AT_LOOP:
        g_at.loop_target = (uint8_t)(val & 0x01u);
        /* Load the active gains for the newly selected loop into the registers */
        s_regs[REG_AT_NEW_KP] = (uint16_t)(MotorCtrl_GetKp(g_at.loop_target) * 1000.0f);
        s_regs[REG_AT_NEW_KI] = (uint16_t)(MotorCtrl_GetKi(g_at.loop_target) * 1000.0f);
        s_regs[REG_AT_NEW_KD] = (uint16_t)(MotorCtrl_GetKd(g_at.loop_target) * 1000.0f);
        send_gains_telemetry(g_at.loop_target);
        break;

    case REG_AT_HYSTERESIS:
        g_at.hysteresis  = (float)(int16_t)val / 100.0f;  /* degrees */
        break;

    case REG_AT_NEW_KP:
        g_at.new_kp      = (float)(int16_t)val / 1000.0f;
        break;

    case REG_AT_NEW_KI:
        g_at.new_ki      = (float)(int16_t)val / 1000.0f;
        break;

    case REG_AT_NEW_KD:
        g_at.new_kd      = (float)(int16_t)val / 1000.0f;
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------
   FC06 — Write Single Register
   ------------------------------------------------------------------------- */
static void handle_fc06(const uint8_t *req, uint16_t len)
{
    if (len < 8) return;
    uint16_t addr = ((uint16_t)req[2] << 8) | req[3];
    uint16_t val  = ((uint16_t)req[4] << 8) | req[5];

    apply_reg_write((uint8_t)addr, val);

    /* Echo the request as response */
    UartDma_SendModbus(req, 8);
}

/* -------------------------------------------------------------------------
   FC16 — Write Multiple Registers
   ------------------------------------------------------------------------- */
static void handle_fc16(const uint8_t *req, uint16_t len)
{
    if (len < 9) return;
    uint16_t start = ((uint16_t)req[2] << 8) | req[3];
    uint16_t count = ((uint16_t)req[4] << 8) | req[5];
    uint8_t  bytes = req[6];

    if (bytes != count * 2u || (start + count) > MB_REG_COUNT) {
        send_exception(MB_FC_WRITEN, 0x02);
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint16_t val = ((uint16_t)req[7 + i*2] << 8) | req[7 + i*2 + 1];
        apply_reg_write((uint8_t)(start + i), val);
    }

    /* Response: [addr][fc][start_hi][start_lo][count_hi][count_lo][crc] */
    uint8_t resp[8];
    resp[0] = MB_ADDR;
    resp[1] = MB_FC_WRITEN;
    resp[2] = req[2]; resp[3] = req[3];
    resp[4] = req[4]; resp[5] = req[5];
    uint16_t crc = crc16(resp, 6);
    resp[6] = (uint8_t)(crc & 0xFFu);
    resp[7] = (uint8_t)(crc >> 8);
    UartDma_SendModbus(resp, 8);
}

/* -------------------------------------------------------------------------
   RX callback — called by uart_dma_manager on LPUART1 IDLE
   ------------------------------------------------------------------------- */
static void on_rx(const uint8_t *data, uint16_t len)
{
    /* Check for Dashboard string commands bypassing Modbus */
    if (len >= 3 && data[0] == 'L' && data[1] == 'A' && data[2] == 'B') {
        char cmd_buf[64] = {0};
        uint16_t copy_len = len < sizeof(cmd_buf) ? len : (sizeof(cmd_buf) - 1);
        memcpy(cmd_buf, data, copy_len);
        for(int i=0; i<copy_len; i++){
            if(cmd_buf[i] == '\r' || cmd_buf[i] == '\n'){
                cmd_buf[i] = '\0';
                break;
            }
        }
        Dashboard_ParseCommand(cmd_buf);
        return;
    }

    if (len < MB_MIN_FRAME) return;
    if (data[0] != MB_ADDR) return;

    /* Verify CRC */
    uint16_t crc_recv = ((uint16_t)data[len-1] << 8) | data[len-2];
    if (crc16(data, len - 2u) != crc_recv) return;

    /* Feed heartbeat watchdog on any valid Modbus packet from PC */
    s_regs[0x00] = HB_HI;

    switch (data[1]) {
    case MB_FC_READ:   if (len >= 8) handle_fc03(data); break;
    case MB_FC_WRITE1: handle_fc06(data, len);           break;
    case MB_FC_WRITEN: handle_fc16(data, len);           break;
    default:           send_exception(data[1], 0x01);   break;
    }

    RobotState.comms.last_rx_len = len;
}

static void send_gains_telemetry(uint8_t loop)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "$GAINS,%u,%ld,%ld,%ld\r\n",
             loop,
             (long)lroundf(MotorCtrl_GetKp(loop) * 1000.0f),
             (long)lroundf(MotorCtrl_GetKi(loop) * 1000.0f),
             (long)lroundf(MotorCtrl_GetKd(loop) * 1000.0f));
    UartDma_SendTelemetry(buf);
}

/* -------------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------------- */
void ModbusBridge_Init(void)
{
    memset(s_regs, 0, sizeof(s_regs));
    s_hb_tick    = 0;
    s_hb_pending = false;

    /* Pre-load current active loop gains (0 = Velocity loop by default) */
    s_regs[REG_AT_LOOP]   = 0;
    s_regs[REG_AT_NEW_KP] = (uint16_t)(MotorCtrl_GetKp(0) * 1000.0f);
    s_regs[REG_AT_NEW_KI] = (uint16_t)(MotorCtrl_GetKi(0) * 1000.0f);
    s_regs[REG_AT_NEW_KD] = (uint16_t)(MotorCtrl_GetKd(0) * 1000.0f);

    UartDma_SetLpuartRxCb(on_rx);

    /* Broadcast initial gains to Web Dashboard */
    send_gains_telemetry(0);
}

uint16_t ModbusBridge_GetReg(uint8_t addr)
{
    return (addr < MB_REG_COUNT) ? s_regs[addr] : 0u;
}

void ModbusBridge_SetReg(uint8_t addr, uint16_t val)
{
    if (addr < MB_REG_COUNT) s_regs[addr] = val;
}

void ModbusBridge_Tick(void)
{
    /* --- Heartbeat: send YA every 200 ms --------------------------------- */
    uint32_t now = HAL_GetTick();
    if (now - s_hb_tick >= HB_PERIOD_MS) {
        s_hb_tick    = now;
        s_hb_pending = false;  /* Self-healing: clear the pending latch so pings are retried if replies get dropped */
        s_regs[0x00] = HB_YA;
        RobotState.comms.heartbeat = HB_YA;
    }

    /* --- Refresh read registers from RobotState ----------------------------- */
    /* 0x26 — bit0=ReedUp, bit1=ReedDown, bit2=ReedClose, bit3=Proximity   */
    uint16_t reeds = 0;
    if (HwIo_GetReedSwitch(REED_UP))    reeds |= 0x01u;
    if (HwIo_GetReedSwitch(REED_DOWN))  reeds |= 0x02u;
    if (HwIo_GetReedSwitch(REED_CLOSE)) reeds |= 0x04u;
    if (HwIo_GetProximity())            reeds |= 0x08u;
    s_regs[0x26] = reeds;

    /* 0x27 — Task (managed by app_main via SetReg) — no update here       */

    /* 0x28–0x30 — Position / Velocity / Acceleration in deg × 10          */
    float pos_rad  = MotorCtrl_GetPosition_rad();
    float vel_rads = RobotState.motion.velocity_rps * (2.0f * 3.14159265f);
    float acc_rads = RobotState.motion.accel_rps2   * (2.0f * 3.14159265f);
    float pos_deg  = pos_rad * (180.0f / 3.14159265f);

    s_regs[0x28] = (uint16_t)(int16_t)(pos_deg  * 10.0f);
    s_regs[0x29] = (uint16_t)(int16_t)(vel_rads * (180.0f / 3.14159265f) * 10.0f);
    s_regs[0x30] = (uint16_t)(int16_t)(acc_rads * (180.0f / 3.14159265f) * 10.0f);

    /* 0x31 — Emergency: bit0=estop, bits15-8=fault_code (for diagnostics)  */
    s_regs[0x31] = (RobotState.sensors.estop ? 0x0001u : 0x0000u)
                 | ((uint16_t)RobotState.comms.fault_code << 8);

    /* --- Auto-Tune READ registers ---------------------------------------- */
    /* 0x3B / 0x3C — mirror volatile fields written by TIM6 ISR              */
    s_regs[REG_AT_STATUS] = (uint16_t)g_at.status;
    s_regs[REG_AT_CYCLES] = g_at.cycles;

    /* --- ApplyGains command (AT_CMD=2) — handled here in main-loop context
       so that MotorCtrl_SetPidGains() runs outside ISR, allowing the brief
       __disable_irq() critical section it uses.                             */
    if (g_at.cmd == AT_CMD_APPLY_GAINS) {
        MotorCtrl_SetPidGains(g_at.loop_target, g_at.new_kp, g_at.new_ki, g_at.new_kd);
        g_at.cmd       = AT_CMD_IDLE;
        s_regs[REG_AT_CMD] = AT_CMD_IDLE;
        send_gains_telemetry(g_at.loop_target);
    }
}

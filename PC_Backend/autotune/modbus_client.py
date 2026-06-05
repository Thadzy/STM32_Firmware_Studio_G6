"""
modbus_client.py — Async Modbus RTU client for auto-tune commands.

Connects to the virtual COM port exposed by serial_bridge.py (typically COM12,
the far end of the com0com pair whose near end is COM11).

New register map (0x33–0x3C; 0x32 = MODBUS_REG_HOME_OFFSET, already occupied):

WRITE (PC → STM32, FC06 / FC16)
──────────────────────────────────────────────────────────
  0x33  AT_CMD          0=Idle  1=StartRelay  2=ApplyGains  3=Abort
  0x34  AT_RELAY_AMP    relay amplitude d, int16 × 10  (e.g. 100 → d=10.0)
  0x35  AT_SETPOINT     operating setpoint, int16 × 10, degrees
  0x36  AT_LOOP         0=velocity inner (1 kHz)  1=position outer (100 Hz)
  0x37  AT_HYSTERESIS   relay dead-band, int16 × 100, degrees
  0x38  AT_NEW_KP       synthesised Kp, int16 × 1000
  0x39  AT_NEW_KI       synthesised Ki, int16 × 1000
  0x3A  AT_NEW_KD       synthesised Kd, int16 × 1000

READ  (STM32 → PC, FC03)
──────────────────────────────────────────────────────────
  0x3B  AT_STATUS       0=Idle  1=Settling  2=RelayActive  3=Done  4=Fault
  0x3C  AT_CYCLES       uint16 — completed oscillation count (firmware-side)

MB_REG_COUNT = 0x3D (61 registers, 0x00..0x3C)

Encoding rules
──────────────
  All int16 values are sent as unsigned 16-bit (two's-complement on the wire).
  Python helper _encode(float, scale) clamps to [-32768, 32767] before masking.
"""

from __future__ import annotations

import logging
from typing import Optional

from pymodbus.client import AsyncModbusSerialClient
from pymodbus.exceptions import ModbusException

log = logging.getLogger(__name__)

SLAVE = 21   # Modbus slave address of the STM32

# ── new register addresses (0x32 is MODBUS_REG_HOME_OFFSET; AT starts at 0x33)
REG_AT_CMD         = 0x33
REG_AT_RELAY_AMP   = 0x34
REG_AT_SETPOINT    = 0x35
REG_AT_LOOP        = 0x36
REG_AT_HYSTERESIS  = 0x37
REG_AT_NEW_KP      = 0x38
REG_AT_NEW_KI      = 0x39
REG_AT_NEW_KD      = 0x3A
REG_AT_STATUS      = 0x3B
REG_AT_CYCLES      = 0x3C

AT_CMD_IDLE        = 0
AT_CMD_START_RELAY = 1
AT_CMD_APPLY_GAINS = 2
AT_CMD_ABORT       = 3

LOOP_VELOCITY = 0
LOOP_POSITION = 1


class AutoTuneModbusClient:
    """
    Thin async wrapper around AsyncModbusSerialClient.

    Serial settings must match serial_bridge.py (230400, 8E1).
    Connect to the *far* end of the com0com pair (the end serial_bridge
    does NOT hold).  If the bridge opened COM11, pass port="COM12" here.
    """

    def __init__(self, port: str = "COM12", baudrate: int = 230400) -> None:
        self._client = AsyncModbusSerialClient(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="E",   # EVEN — must match serial_bridge.py / STM32 UART config
            stopbits=1,
            timeout=0.5,
        )
        self._connected = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        try:
            await self._client.connect()
            self._connected = bool(self._client.connected)
            log.info("Modbus %s on %s", "connected" if self._connected else "FAILED", self._client.comm_params.host)
        except Exception as exc:
            log.error("Modbus connect error: %s", exc)
            self._connected = False
        return self._connected

    async def disconnect(self) -> None:
        if self._connected:
            self._client.close()
            self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Public commands
    # ------------------------------------------------------------------

    async def start_relay(
        self,
        amplitude: float,
        setpoint_deg: float,
        hysteresis_deg: float,
        loop_target: int = LOOP_POSITION,
    ) -> bool:
        """
        Write relay parameters then assert AT_CMD=StartRelay in a single
        FC16 (write-multiple) burst to keep the register state coherent.
        """
        values = [
            _encode(amplitude,      scale=10),
            _encode(setpoint_deg,   scale=10),
            loop_target & 0xFFFF,
            _encode(hysteresis_deg, scale=100),
        ]
        ok = await self._write_multiple(REG_AT_RELAY_AMP, values)
        if ok:
            ok = await self._write_single(REG_AT_CMD, AT_CMD_START_RELAY)
        return ok

    async def apply_gains(self, kp: float, ki: float, kd: float) -> bool:
        """Write new PID gains and assert AT_CMD=ApplyGains."""
        values = [
            _encode(kp, scale=1000),
            _encode(ki, scale=1000),
            _encode(kd, scale=1000),
        ]
        ok = await self._write_multiple(REG_AT_NEW_KP, values)
        if ok:
            ok = await self._write_single(REG_AT_CMD, AT_CMD_APPLY_GAINS)
        return ok

    async def abort(self) -> bool:
        return await self._write_single(REG_AT_CMD, AT_CMD_ABORT)

    async def read_status(self) -> Optional[dict]:
        """Return {'status': int, 'cycles': int} or None on error."""
        regs = await self._read(REG_AT_STATUS, count=2)
        if regs is None:
            return None
        return {"status": regs[0], "cycles": regs[1]}

    async def send_heartbeat(self) -> bool:
        """Reply 18537 ("HI") to register 0x00 heartbeat."""
        return await self._write_single(0x00, 18537)

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------

    async def _write_single(self, address: int, value: int) -> bool:
        try:
            r = await self._client.write_register(address, value & 0xFFFF, slave=SLAVE)
            if r.isError():
                log.warning("FC06 error @ 0x%02X: %s", address, r)
                return False
            return True
        except ModbusException as exc:
            log.error("FC06 exception @ 0x%02X: %s", address, exc)
            return False

    async def _write_multiple(self, start: int, values: list[int]) -> bool:
        try:
            r = await self._client.write_registers(start, [v & 0xFFFF for v in values], slave=SLAVE)
            if r.isError():
                log.warning("FC16 error @ 0x%02X: %s", start, r)
                return False
            return True
        except ModbusException as exc:
            log.error("FC16 exception @ 0x%02X: %s", start, exc)
            return False

    async def _read(self, address: int, count: int = 1) -> Optional[list[int]]:
        try:
            r = await self._client.read_holding_registers(address, count, slave=SLAVE)
            if r.isError():
                log.warning("FC03 error @ 0x%02X: %s", address, r)
                return None
            return r.registers
        except ModbusException as exc:
            log.error("FC03 exception @ 0x%02X: %s", address, exc)
            return None


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _encode(value: float, scale: int) -> int:
    """Encode float to signed int16 wire value (two's-complement, unsigned repr)."""
    raw = int(round(value * scale))
    raw = max(-32768, min(32767, raw))
    return raw & 0xFFFF

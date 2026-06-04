#!/usr/bin/env python3
"""
serial_bridge.py  —  Phase 7: USB serial demultiplexer

The robot STM32 shares one USB-serial port for two streams:

  $...\r\n  telemetry lines  →  WebSocket broadcast on port 8765  (React dashboard)
  Modbus RTU frames          →  CRC-validate, forward to COM10  (main.exe on COM11)

main.exe writes Modbus commands on COM11; the bridge reads COM10 and forwards
those commands to the robot, then routes the robot's Modbus responses back.

Usage:
    python serial_bridge.py [ROBOT_PORT]      # e.g. python serial_bridge.py COM5
    python serial_bridge.py                   # auto-scan all COM ports

Requirements:
    pip install pyserial websockets

Notes:
  - COM10/COM11 must be a virtual COM pair (e.g. com0com).  The bridge opens
    COM10; main.exe opens COM11.  If COM10 is unavailable the Modbus passthrough
    is disabled but telemetry → WebSocket still works.
  - Stop any other program (test_robot.py, base system) using the robot port
    before running this bridge.
"""

import sys
import time
import threading
import queue
import asyncio

try:
    import websockets
except ImportError:
    print("ERROR: 'websockets' not installed.  Run:  pip install websockets")
    sys.exit(1)

import serial
import serial.tools.list_ports
from typing import Optional

# ── configuration ──────────────────────────────────────────────────────────
_ROBOT_PORT: Optional[str] = sys.argv[1] if len(sys.argv) > 1 else None
COM10_PORT   = "AUTO"    # auto-detect: tries COM11 then COM12, uses whichever is free
WS_PORT      = 8765
BAUD         = 230400
PARITY       = serial.PARITY_EVEN
SLAVE_ADDR   = 21        # 0x15  Modbus slave address of the robot

# ── CRC-16 (Modbus) ────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _crc_ok(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    expected = _crc16(frame[:-2])
    received = frame[-2] | (frame[-1] << 8)
    return expected == received


# ── serial read helper ─────────────────────────────────────────────────────
def _read_n(ser: serial.Serial, n: int) -> Optional[bytes]:
    """Read exactly n bytes; return None if the port times out before that."""
    buf = bytearray()
    deadline = time.monotonic() + 0.2
    while len(buf) < n:
        if time.monotonic() > deadline:
            return None
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


# ── shared queues & state ──────────────────────────────────────────────────
_ws_q:      "queue.Queue[str]"   = queue.Queue()   # telemetry lines → WS clients
_to_com10:  "queue.Queue[bytes]" = queue.Queue()   # robot responses → COM10
_to_robot:  "queue.Queue[bytes]" = queue.Queue()   # commands from COM10 → robot
_stop       = threading.Event()
_ws_clients: set = set()
_ws_lock    = threading.Lock()


# ── robot reader thread ────────────────────────────────────────────────────
def _robot_reader(ser: serial.Serial) -> None:
    """
    Reads robot serial indefinitely.
      '$'          → read ASCII line → put in _ws_q (WebSocket telemetry)
      SLAVE_ADDR   → read Modbus response, CRC-check, put in _to_com10
    """
    while not _stop.is_set():
        b1 = ser.read(1)
        if not b1:
            continue

        b = b1[0]

        # ── telemetry line ────────────────────────────────────────────────
        if b == 0x24:   # '$'
            rest = ser.readline()
            line = '$' + rest.decode('ascii', errors='ignore').strip()
            _ws_q.put(line)
            continue

        # ── Modbus response from robot ────────────────────────────────────
        if b != SLAVE_ADDR:
            continue    # stray / noise byte; discard

        fc_b = ser.read(1)
        if not fc_b:
            continue
        fc = fc_b[0]

        frame: Optional[bytes] = None

        if fc & 0x80:
            # Exception response: [addr, fc|0x80, exc_code, crc_lo, crc_hi]  (5 bytes)
            tail = _read_n(ser, 3)
            if tail:
                frame = bytes([b, fc]) + tail

        elif fc == 0x03:
            # Read holding registers: [addr, fc, byte_count, data…, crc_lo, crc_hi]
            bc_b = ser.read(1)
            if bc_b:
                bc = bc_b[0]
                tail = _read_n(ser, bc + 2)   # data + 2 CRC bytes
                if tail:
                    frame = bytes([b, fc, bc]) + tail

        elif fc in (0x06, 0x10):
            # Write single / write multiple echo: fixed 8 bytes total
            tail = _read_n(ser, 6)
            if tail:
                frame = bytes([b, fc]) + tail

        else:
            print(f"[bridge] Unknown FC=0x{fc:02X} in robot response — ignoring")
            continue

        if frame is None:
            continue

        if _crc_ok(frame):
            _to_com10.put(frame)
        else:
            print(f"[bridge] Bad CRC in robot response (FC=0x{fc:02X}, {len(frame)} bytes)"
                  f" — dropped")


# ── robot writer thread ────────────────────────────────────────────────────
def _robot_writer(ser: serial.Serial) -> None:
    """Drain _to_robot queue and write Modbus commands to the robot."""
    while not _stop.is_set():
        try:
            frame = _to_robot.get(timeout=0.1)
            ser.write(frame)
        except queue.Empty:
            pass


# ── COM10 reader thread ────────────────────────────────────────────────────
def _com10_reader(ser: serial.Serial) -> None:
    """
    Read Modbus commands from COM10 (main.exe on COM11) → _to_robot queue.
    main.exe is trusted so we forward without CRC validation.
    """
    while not _stop.is_set():
        b1 = ser.read(1)
        if not b1:
            continue

        b = b1[0]
        if b != SLAVE_ADDR:
            continue

        fc_b = ser.read(1)
        if not fc_b:
            continue
        fc = fc_b[0]

        frame: Optional[bytes] = None

        if fc == 0x03:
            # FC03 read request: fixed 8 bytes (addr fc start_hi start_lo cnt_hi cnt_lo crc×2)
            tail = _read_n(ser, 6)
            if tail:
                frame = bytes([b, fc]) + tail

        elif fc == 0x06:
            # FC06 write single: fixed 8 bytes
            tail = _read_n(ser, 6)
            if tail:
                frame = bytes([b, fc]) + tail

        elif fc == 0x10:
            # FC16 write multiple: variable length
            # [addr fc start_hi start_lo count_hi count_lo byte_count data… crc_lo crc_hi]
            header = _read_n(ser, 5)   # start(2) + count(2) + byte_count(1)
            if header:
                bc = header[4]
                tail = _read_n(ser, bc + 2)   # data + 2 CRC bytes
                if tail:
                    frame = bytes([b, fc]) + header + tail

        else:
            print(f"[bridge] Unknown FC=0x{fc:02X} from main.exe — ignoring")
            continue

        if frame:
            _to_robot.put(frame)


# ── COM10 writer thread ────────────────────────────────────────────────────
def _com10_writer(ser: serial.Serial) -> None:
    """Drain _to_com10 queue and write robot responses back to COM10."""
    while not _stop.is_set():
        try:
            frame = _to_com10.get(timeout=0.1)
            ser.write(frame)
        except queue.Empty:
            pass


# ── WebSocket server ───────────────────────────────────────────────────────
async def _ws_handler(ws, path=None) -> None:
    """Accept a WebSocket client and keep the connection alive."""
    addr = getattr(ws, 'remote_address', '?')
    print(f"[ws] Client connected:    {addr}")
    with _ws_lock:
        _ws_clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        with _ws_lock:
            _ws_clients.discard(ws)
        print(f"[ws] Client disconnected: {addr}")


async def _ws_broadcaster() -> None:
    """Drain _ws_q every 5 ms and broadcast to all connected WebSocket clients."""
    while True:
        lines = []
        while True:
            try:
                lines.append(_ws_q.get_nowait())
            except queue.Empty:
                break

        if lines:
            msg = '\n'.join(lines)
            with _ws_lock:
                clients = set(_ws_clients)
            dead = set()
            for c in clients:
                try:
                    await c.send(msg)
                except Exception:
                    dead.add(c)
            if dead:
                with _ws_lock:
                    _ws_clients.difference_update(dead)

        await asyncio.sleep(0.005)


async def _ws_main() -> None:
    async with websockets.serve(_ws_handler, "0.0.0.0", WS_PORT):
        print(f"[ws] Server ready  →  ws://localhost:{WS_PORT}")
        await _ws_broadcaster()


# ── port auto-scan ─────────────────────────────────────────────────────────
def _make_ping() -> bytes:
    payload = bytes([SLAVE_ADDR, 0x03, 0x00, 0x27, 0x00, 0x01])
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])


def _scan_for_robot() -> Optional[str]:
    ping = _make_ping()
    for p in sorted(serial.tools.list_ports.comports()):
        if p.device == COM10_PORT:
            continue
        try:
            with serial.Serial(p.device, BAUD, parity=PARITY, timeout=0.3) as s:
                s.reset_input_buffer()
                s.write(ping)
                r = s.read(7)
                if len(r) == 7 and r[1] == 0x03:
                    return p.device
        except Exception:
            pass
    return None


# ── main ───────────────────────────────────────────────────────────────────
def main() -> None:
    global _ROBOT_PORT

    print("=" * 60)
    print("  serial_bridge.py  —  Phase 7")
    print("=" * 60)

    print("\nAvailable COM ports:")
    for p in sorted(serial.tools.list_ports.comports()):
        print(f"  {p.device:<12} {p.description}")

    # Locate robot serial port
    if _ROBOT_PORT is None:
        print("\nAuto-scanning for robot...")
        _ROBOT_PORT = _scan_for_robot()
        if _ROBOT_PORT is None:
            print("[ERROR] Robot not found on any port.")
            print("  • Make sure the robot is powered and no other program holds the port.")
            print("  • Or specify the port:  python serial_bridge.py COM5")
            sys.exit(1)
        print(f"  Robot found on {_ROBOT_PORT}")

    # Open robot serial
    try:
        ser_robot = serial.Serial(_ROBOT_PORT, BAUD, parity=PARITY, timeout=0.05)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {_ROBOT_PORT}: {e}")
        sys.exit(1)

    # Open virtual COM port for main.exe passthrough (optional)
    ser_com10: Optional[serial.Serial] = None
    loopback_port: Optional[str] = None
    basesys_port:  Optional[str] = None

    candidates = ["COM11", "COM12"] if COM10_PORT == "AUTO" else [COM10_PORT]
    for candidate in candidates:
        if candidate == _ROBOT_PORT:
            continue
        try:
            ser_com10 = serial.Serial(candidate, BAUD, parity=PARITY, timeout=0.05)
            loopback_port = candidate
            # Tell user which port to give to main.exe (the other end of the pair)
            pair = {"COM11": "COM12", "COM12": "COM11"}
            basesys_port = pair.get(candidate, "the other com0com port")
            break
        except serial.SerialException:
            pass

    if ser_com10:
        print(f"\n  Modbus passthrough active!")
        print(f"  Bridge  →  {loopback_port}")
        print(f"  *** Configure base system / main.exe to use: {basesys_port} ***")
    else:
        print(f"\n[WARN] No virtual COM port available (tried: {candidates})")
        print("  Modbus passthrough disabled — telemetry-only mode.")
        print("  Close any program holding COM11/COM12 and retry.")

    print(f"\n  Robot port  : {_ROBOT_PORT}")
    print(f"  WebSocket   : ws://localhost:{WS_PORT}")
    print("\nPress Ctrl+C to stop.\n")

    # Launch I/O threads
    threading.Thread(
        target=_robot_reader, args=(ser_robot,), daemon=True, name="robot-rx"
    ).start()
    threading.Thread(
        target=_robot_writer, args=(ser_robot,), daemon=True, name="robot-tx"
    ).start()

    if ser_com10:
        threading.Thread(
            target=_com10_reader, args=(ser_com10,), daemon=True, name="com10-rx"
        ).start()
        threading.Thread(
            target=_com10_writer, args=(ser_com10,), daemon=True, name="com10-tx"
        ).start()

    # Run WebSocket server — blocks until Ctrl+C
    try:
        asyncio.run(_ws_main())
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[bridge] Shutting down...")
        _stop.set()
        try:
            ser_robot.close()
        except Exception:
            pass
        if ser_com10:
            try:
                ser_com10.close()
            except Exception:
                pass
        print("[bridge] Stopped.")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
serial_bridge.py  —  Phase 7: USB serial demultiplexer

The robot STM32 shares one USB-serial port for two streams:

  $...\r\n  telemetry lines  ->  WebSocket broadcast on port 8765  (React dashboard)
  Modbus RTU frames          ->  CRC-validate, forward to COM10  (main.exe on COM11)

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
    is disabled but telemetry -> WebSocket still works.
  - Stop any other program (test_robot.py, base system) using the robot port
    before running this bridge.
"""

import sys
import time
import threading
import queue
import asyncio
import json

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
DEBUG        = True      # print telemetry + frame logs to console
LOG_RATE_S   = 2.0       # min seconds between repeated BAD-CRC log lines

# ── Heartbeat proxy ──────────────────────────────────────────────────────────
# The base system does not send the HI (18537) reply to the robot's YA (22881)
# heartbeat.  Without it the firmware's 2-second watchdog fires continuously
# (fault_code=0x20), which the base system interprets as a fault and blocks
# the Auto command.  The bridge handles the heartbeat reply on behalf of the PC.
HB_YA              = 22881   # 'YA' — robot sends this every 200 ms
HB_HI              = 18537   # 'HI' — PC must reply with this
HB_SEND_INTERVAL_S = 0.4     # send HI every 400 ms (well inside 2000 ms timeout)

_REG_NAMES = {
    # ── WRITE registers (PC -> robot) ────────────────────────────────────────
    0x00: 'heartbeat',
    0x01: 'mode',           # 1=Home 2=Jog 4=Auto 8=SetHome 16=Test
    0x02: 'manual_gripper', # 0=Up 1=Down 2=Open 4=Close
    0x03: 'gripper_seq',    # 1=Pick 2=Place
    0x04: 'gripper_auto_en',# bit0 enable
    0x05: 'jog_step_deg',   # int16 degrees; +CCW -CW
    0x06: 'test_type',
    0x07: 'perf_vel',
    0x08: 'perf_acc',
    0x09: 'prec_init_pos',
    0x10: 'prec_final_pos',
    0x11: 'prec_repeat',
    # 0x12–0x21: pick/place sequence slots (int16: magnitude=index, sign=direction)
    0x12: 'slot0_pick1',
    0x13: 'slot1_place1',
    0x14: 'slot2_pick2',
    0x15: 'slot3_place2',
    0x16: 'slot4_pick3',
    0x17: 'slot5_place3',
    0x18: 'slot6_pick4',
    0x19: 'slot7_place4',
    0x1A: 'slot8_pick5',
    0x1B: 'slot9_place5',
    0x1C: 'slot10_pick6',
    0x1D: 'slot11_place6',
    0x1E: 'slot12_pick7',
    0x1F: 'slot13_place7',
    0x20: 'slot14_pick8',
    0x21: 'slot15_place8',
    0x22: 'n_pairs',        # number of pick+place pairs (max 8)
    0x23: 'p2p_unit',       # 0=degree 1=index
    0x24: 'p2p_target',     # int16 target
    0x25: 'soft_stop',      # bit0
    # ── READ registers (robot -> PC) ─────────────────────────────────────────
    0x26: 'reed_sensors',   # bit0=Reed1 bit1=Reed2 bit2=Reed3(jaw)
    0x27: 'task',           # bit0=Homing 1=GoPick 2=GoPlace 3=GoPoint
    0x28: 'pos_deg_x10',
    0x29: 'vel_dps_x10',
    0x30: 'acc_dps2_x10',
    0x31: 'emergency',      # bit0=estop bits15-8=fault_code
    0x32: 'home_offset_deg',# int16 fine-tune offset in whole degrees
    0x33: 'at_cmd',         # 0=Idle 1=Start 2=Apply 3=Abort
    0x34: 'at_relay_amp',   # relay amplitude
    0x35: 'at_setpoint',    # operating setpoint (deg)
    0x36: 'at_loop',        # 0=Velocity 1=Position
    0x37: 'at_hysteresis',  # dead-band (deg)
    0x38: 'at_new_kp',      # tuned Kp
    0x39: 'at_new_ki',      # tuned Ki
    0x3A: 'at_new_kd',      # tuned Kd
    0x3B: 'at_status',      # status (0=Idle ... 4=Fault)
    0x3C: 'at_cycles',      # completed oscillation cycles
}

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


def _make_fc06(reg: int, val: int) -> bytes:
    """Build a valid Modbus FC06 write-single-register frame."""
    payload = bytes([SLAVE_ADDR, 0x06,
                     (reg >> 8) & 0xFF, reg & 0xFF,
                     (val >> 8) & 0xFF, val & 0xFF])
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])


# ── heartbeat proxy thread ────────────────────────────────────────────────────
def _heartbeat_thread() -> None:
    """
    Send HI (18537) to robot reg 0x00 every HB_SEND_INTERVAL_S.
    This keeps the firmware heartbeat watchdog alive when the base system
    does not implement the YA/HI reply protocol.
    The robot echoes every FC06 write; _robot_reader absorbs those echoes
    using _hb_pending so the base system never sees them.
    """
    global _hb_pending
    _last_hb_log = 0.0
    while not _stop.is_set():
        frame = _make_fc06(0x00, HB_HI)
        with _hb_lock:
            _hb_pending += 1
        _to_robot.put(frame)
        _stop.wait(HB_SEND_INTERVAL_S)


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
_ws_q:      "queue.Queue[str]"   = queue.Queue()   # telemetry lines -> WS clients
_to_com10:  "queue.Queue[bytes]" = queue.Queue()   # robot responses -> COM10
_to_robot:  "queue.Queue[bytes]" = queue.Queue()   # commands from COM10 -> robot
_stop       = threading.Event()
_ws_clients: set = set()
_ws_lock    = threading.Lock()

# Heartbeat proxy: count of bridge-injected HI writes awaiting robot echo.
# The echo is absorbed so it is not forwarded to the base system on COM10.
_hb_lock    = threading.Lock()
_hb_pending = 0


# ── HTTP Server for Dashboard ────────────────────────────────────────────────
def _http_server_thread() -> None:
    import http.server
    import socketserver

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # Suppress request spam in the console

    port = 8000
    while port < 8010:
        try:
            socketserver.TCPServer.allow_reuse_address = True
            with socketserver.TCPServer(("", port), QuietHandler) as httpd:
                print(f"[http] Dashboard served at -> http://localhost:{port}/controller_dashboard.html")
                httpd.serve_forever()
        except OSError:
            port += 1


# ── robot reader thread ────────────────────────────────────────────────────
def _robot_reader(ser: serial.Serial) -> None:
    """
    Reads robot serial indefinitely.
      '$'          -> read ASCII line -> put in _ws_q (WebSocket telemetry)
      SLAVE_ADDR   -> read Modbus response, CRC-check, put in _to_com10
    """
    _bad_count  = 0
    _last_log_t = 0.0   # last time a BAD-CRC line was printed
    _last_tel_t = 0.0   # last time telemetry was printed to console
    _last_rx_t  = 0.0   # last time Modbus RX was printed to console
    _last_fault = None
    _last_estop = None
    _last_fsm_state = None
    _last_run_mode  = None
    _last_fault_code = None
    _ctrl_was_active = False
    _tel_was_active  = False

    try:
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
                if DEBUG:
                    now = time.monotonic()
                    _FSM = {0:'INIT', 1:'HOMING', 2:'IDLE', 3:'RUNNING', 4:'FAULT'}
                    _FAULT_DESCS = {
                        0x00: 'None',
                        0x01: 'Hardware E-Stop Activated',
                        0x02: 'Homing: Edge B Not Found',
                        0x03: 'Homing: Sensor Never Cleared',
                        0x04: 'Homing: Sensor Not Found',
                        0x10: 'Joystick Emergency Tripped',
                        0x20: 'Heartbeat Watchdog Timeout',
                        0x40: 'Safety: Encoder Cable Disconnect',
                        0x41: 'Safety: Cable Limit Exceeded',
                        0x42: 'Safety: Overcurrent Tripped',
                        0x43: 'Safety: Position Tracking Jam'
                    }
                    if line.startswith('$ST,'):
                        p = line.split(',')
                        if len(p) >= 4:
                            fsm = int(p[1]) if p[1].lstrip('-').isdigit() else 0
                            fsm_n = _FSM.get(fsm, f'?{fsm}')
                            run_mode = int(p[2]) if p[2].lstrip('-').isdigit() else 0
                            fault = int(p[3]) if p[3].lstrip('-').isdigit() else 0
                            fault_desc = _FAULT_DESCS.get(fault, f'Unknown 0x{fault:02X}')
                            if (fsm != _last_fsm_state) or (run_mode != _last_run_mode) or (fault != _last_fault_code):
                                _last_fsm_state = fsm
                                _last_run_mode = run_mode
                                _last_fault_code = fault
                                alert = '  *** ROBOT IN FAULT — press RESET button on hardware ***' \
                                        if fsm == 4 else ''
                                print(f"[STATE] FSM={fsm_n:<7} | run_mode={run_mode} | fault=0x{fault:02X} ({fault_desc}){alert}")
                        else:
                            print(f"[STATE] {line}")
                    elif line.startswith('$AUTO,'):
                        p = line.split(',')
                        if len(p) >= 2 and p[1] == 'DONE':
                            print(f"[AUTO] Sequence DONE after {p[2] if len(p)>2 else '?'} steps")
                        elif len(p) >= 4:
                            step = p[1]
                            raw  = int(p[2])
                            tgt  = int(p[3]) / 10.0
                            pick = (int(step) % 2 == 0)
                            print(f"[AUTO] step={step} ({'PICK' if pick else 'PLACE'})  raw={raw}  target={tgt:.1f}°")
                        else:
                            print(f"[AUTO] {line}")
                    elif line.startswith('$HOM,'):
                        print(f"[HOMING] {line[5:]}")
                    elif line.startswith('$GAINS,'):
                        p = line.split(',')
                        if len(p) >= 5:
                            loop = int(p[1])
                            loop_name = "Velocity (Inner)" if loop == 0 else "Position (Outer)"
                            kp = int(p[2]) / 1000.0
                            ki = int(p[3]) / 1000.0
                            kd = int(p[4]) / 1000.0
                            print(f"[GAINS] {loop_name} Loop | Kp={kp:.3f} | Ki={ki:.3f} | Kd={kd:.3f}")
                    elif line.startswith('$CTRL,'):
                        p = line.split(',')
                        if len(p) >= 13:
                            tick  = int(p[1])
                            tgt   = int(p[2]) / 10.0
                            pos   = int(p[3]) / 10.0
                            shp   = int(p[4]) / 10.0
                            p_act = int(p[5]) / 100.0
                            p_err = int(p[6]) / 100.0
                            v_cmd = int(p[7]) / 10.0
                            v_act = int(p[8]) / 10.0
                            v_err = int(p[9]) / 10.0
                            pwm   = int(p[10]) / 10.0
                            pos_i = int(p[11]) / 10.0
                            spd_i = int(p[12]) / 10.0
                            
                            is_active = (abs(v_cmd) > 0.1) or (abs(v_act) > 0.1) or (abs(pwm) > 1.0) or (_last_fsm_state in (1, 3))
                            if is_active:
                                _ctrl_was_active = True
                                if now - _last_tel_t >= 0.5:
                                    _last_tel_t = now
                                    print(f"[CTRL] Tick={tick:<6} | Pos Tgt/Shp/Act: {tgt:>5.1f}° / {shp:>5.1f}° / {p_act:>6.2f}° (Err: {p_err:>5.2f}°)\n"
                                          f"       PWM={pwm:>6.1f}% | Vel Cmd/Act: {v_cmd:>6.1f}°/s / {v_act:>6.1f}°/s (Err: {v_err:>5.1f}°/s)\n"
                                          f"       PID Integrals | Pos: {pos_i:>5.2f} | Spd: {spd_i:>5.2f}")
                            elif _ctrl_was_active:
                                _ctrl_was_active = False
                                print(f"[CTRL] Motor Stopped | Tick={tick:<6} | Pos={p_act:>5.1f}°")
                    elif line.startswith('$T,'):
                        p = line.split(',')
                        if len(p) >= 6:
                            tick = int(p[1])
                            pos  = int(p[2]) / 10.0
                            vel  = int(p[3]) / 10.0
                            acc  = int(p[4]) / 10.0
                            pwm  = int(p[5]) / 10.0
                            
                            is_active = (abs(vel) > 0.1) or (abs(pwm) > 1.0) or (_last_fsm_state in (1, 3))
                            if is_active:
                                _tel_was_active = True
                                if now - _last_tel_t >= 0.5:
                                    _last_tel_t = now
                                    print(f"[TELE] Tick={tick:<6} | Pos={pos:>5.1f}° | Vel={vel:>5.1f}°/s | Acc={acc:>5.1f}°/s² | PWM={pwm:>5.1f}%")
                            elif _tel_was_active:
                                _tel_was_active = False
                                print(f"[TELE] Motor Stopped | Tick={tick:<6} | Pos={pos:>5.1f}°")
                    elif now - _last_tel_t >= 0.5:
                        _last_tel_t = now
                        print(f"[tel] {line}")
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
                _bad_count = 0

                # ── absorb FC06 echo for bridge-injected HI heartbeat writes ──
                # The robot echoes every FC06 write verbatim.  If we injected a
                # HI write, consume that echo silently so the base system never
                # sees a FC06 frame it didn't send (which would confuse it).
                if fc == 0x06 and len(frame) == 8:
                    _reg_addr = (frame[2] << 8) | frame[3]
                    _reg_val  = (frame[4] << 8) | frame[5]
                    if _reg_addr == 0x00 and _reg_val == HB_HI:
                        with _hb_lock:
                            global _hb_pending
                            if _hb_pending > 0:
                                _hb_pending -= 1
                                continue   # drop echo — not forwarded to COM10

                _to_com10.put(frame)
                if DEBUG and fc == 0x03 and len(frame) == 105:
                    def _reg(addr):
                        o = 3 + addr * 2
                        return (frame[o] << 8) | frame[o + 1]
                    hb   = _reg(0x00)
                    emg  = _reg(0x31)
                    estop  = bool(emg & 0x01)
                    fcode  = emg >> 8
                    _FAULT_DESC = {
                        0x00: 'none',
                        0x01: 'estop trip',
                        0x02: 'homing: edge-B not found',
                        0x03: 'homing: sensor stuck ON',
                        0x04: 'homing: sensor not found in sweep',
                        0x10: 'joystick emergency',
                        0x20: 'prev heartbeat timeout (harmless — link now OK)',
                    }
                    fdesc = _FAULT_DESC.get(fcode, f'unknown 0x{fcode:02X}')
                    
                    if fcode != _last_fault or estop != _last_estop:
                        _last_fault = fcode
                        _last_estop = estop
                        blocking = fcode not in (0, 0x20)
                        print(f"[STATUS] fault={fdesc} | estop={'YES !!!' if estop else 'no'}"
                              f"{'  -> PRESS RESET BTN on hardware' if blocking else ''}")
            else:
                _bad_count += 1
                now = time.monotonic()
                if now - _last_log_t >= LOG_RATE_S:
                    _last_log_t = now
                    exp  = _crc16(frame[:-2])
                    recv = frame[-2] | (frame[-1] << 8)
                    echo_hint = " ← looks like TX echo of request!" if len(frame) <= 8 else ""
                    print(f"[rx] BAD CRC FC=0x{fc:02X} {len(frame)}B"
                          f"  exp=0x{exp:04X} got=0x{recv:04X}"
                          f"  (×{_bad_count} since last log){echo_hint}")
                    print(f"     bytes: {frame.hex(' ')}")
    except (serial.SerialException, ValueError, TypeError):
        # Shutdown or port closed
        if not _stop.is_set():
            print("[bridge] Robot serial connection lost.")
    except Exception as e:
        if not _stop.is_set():
            print(f"[bridge] robot-rx error: {e}")


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
    Read Modbus commands from COM10 (main.exe on COM11) -> _to_robot queue.
    main.exe is trusted so we forward without CRC validation.
    """
    _last_written_val = {}
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
            if DEBUG and fc == 0x06:
                reg = (frame[2] << 8) | frame[3]
                val = (frame[4] << 8) | frame[5]
                if reg != 0x00:  # Suppress heartbeat logs to clean up console
                    last_val = _last_written_val.get(reg)
                    if last_val != val:
                        _last_written_val[reg] = val
                        desc = _REG_NAMES.get(reg, f'reg0x{reg:02X}')
                        note = ''
                        if reg == 0x01 and val == 1:     note = '  ← HOME command'
                        elif reg == 0x01:                note = f'  ← mode={val}'
                        print(f"[tx] FC06 {desc}={val}{note}")
            elif DEBUG and fc == 0x10:
                # [addr fc start_hi start_lo count_hi count_lo bc data… crc lo hi]
                start = (frame[2] << 8) | frame[3]
                count = (frame[4] << 8) | frame[5]
                data  = frame[7:7 + count * 2]
                vals  = [ (data[i] << 8) | data[i + 1] for i in range(0, len(data), 2) ]
                # Check if any value changed
                changed = False
                for k, v in enumerate(vals):
                    reg = start + k
                    if _last_written_val.get(reg) != v:
                        _last_written_val[reg] = v
                        changed = True
                if changed:
                    # show signed too, since slots are int16 (sign = direction)
                    def _s16(v): return v - 0x10000 if v >= 0x8000 else v
                    pairs = ', '.join(
                        f'0x{start+k:02X}={v}({_s16(v)})' for k, v in enumerate(vals))
                    print(f"[tx] FC16 write {count} regs from 0x{start:02X}: {pairs}")
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
    """Accept a WebSocket client, process incoming commands, and handle disconnect."""
    addr = getattr(ws, 'remote_address', '?')
    print(f"[ws] Client connected:    {addr}")
    with _ws_lock:
        _ws_clients.add(ws)
    try:
        async for msg_str in ws:
            try:
                data = json.loads(msg_str)
                action = data.get("action")
                if action == "write_reg":
                    reg = data.get("reg")
                    val = data.get("val")
                    if isinstance(reg, int) and isinstance(val, int):
                        reg &= 0xFFFF
                        val &= 0xFFFF
                        frame = _make_fc06(reg, val)
                        _to_robot.put(frame)
                        if DEBUG:
                            desc = _REG_NAMES.get(reg, f'reg0x{reg:02X}')
                            print(f"[ws] Client {addr} write reg 0x{reg:02X} ({desc}) = {val}")
                elif action == "cmd":
                    cmd_str = data.get("cmd", "")
                    if cmd_str:
                        _to_robot.put(cmd_str.encode('ascii') + b'\r\n')
                        if DEBUG: print(f"[ws] Client {addr} sent string cmd: {cmd_str}")
            except Exception as e:
                print(f"[ws] Error parsing client message: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
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
        print(f"[ws] Server ready  ->  ws://localhost:{WS_PORT}")
        await _ws_broadcaster()


# ── port detection helpers ─────────────────────────────────────────────────
def _make_ping() -> bytes:
    payload = bytes([SLAVE_ADDR, 0x03, 0x00, 0x27, 0x00, 0x01])
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])


def _find_stm32_port() -> Optional[str]:
    """Find STM32 STLink VCP by USB description — no pinging, no false positives."""
    for p in sorted(serial.tools.list_ports.comports()):
        if "STMicroelectronics" in p.description or "STLink" in p.description:
            return p.device
    return None


def _find_com0com_pair(exclude: str) -> tuple:
    """Return (bridge_side, mainexe_side) from the com0com virtual pair."""
    ports = sorted(
        p.device for p in serial.tools.list_ports.comports()
        if "com0com" in p.description.lower() and p.device != exclude
    )
    if len(ports) >= 2:
        return ports[0], ports[1]
    if len(ports) == 1:
        return ports[0], "??"
    return None, None


def _validate_robot(port: str) -> bool:
    """
    Open port, send a read-reg-0x27 ping, wait up to 1 s for a valid
    FC=0x03 response.  Returns True only if the STM32 answers correctly.
    """
    ping = _make_ping()
    try:
        with serial.Serial(port, BAUD, parity=PARITY, timeout=1.0) as s:
            s.reset_input_buffer()
            s.write(ping)
            raw = s.read(200)   # read whatever arrives in 1 s
        # Scan for a valid Modbus FC=0x03 response in the buffer
        for i in range(len(raw) - 4):
            if raw[i] != SLAVE_ADDR or raw[i + 1] != 0x03:
                continue
            bc = raw[i + 2]
            end = i + 3 + bc + 2
            if end > len(raw):
                continue
            frame = raw[i:end]
            if _crc_ok(frame):
                return True
    except Exception:
        pass
    return False


# ── main ───────────────────────────────────────────────────────────────────
def main() -> None:
    global _ROBOT_PORT

    print("=" * 60)
    print("  serial_bridge.py  —  Phase 7")
    print("=" * 60)

    # ── 1. List available ports ────────────────────────────────────────────
    all_ports = sorted(serial.tools.list_ports.comports())
    print("\nAvailable COM ports:")
    for p in all_ports:
        print(f"  {p.device:<12} {p.description}")

    # ── 2. Locate robot port ───────────────────────────────────────────────
    if _ROBOT_PORT is None:
        robot_port = _find_stm32_port()
        if robot_port:
            print(f"\n[OK] STM32 found by description -> {robot_port}")
        else:
            print("\n[WARN] No STMicroelectronics port found — falling back to ping scan...")
            robot_port = next(
                (p.device for p in all_ports
                 if "com0com" not in p.description.lower()
                 and _validate_robot(p.device)),
                None
            )
        if robot_port is None:
            print("\n[ERROR] Robot not found.")
            print("  Fix: plug in the STM32, wait for Windows to assign a COM port,")
            print("  then retry.  Or run:  python serial_bridge.py COMx")
            sys.exit(1)
        _ROBOT_PORT = robot_port
    else:
        print(f"\n[OK] Using user-specified port: {_ROBOT_PORT}")

    # ── 3. Validate the robot connection ───────────────────────────────────
    print(f"  Validating {_ROBOT_PORT}...", end=" ", flush=True)
    if _validate_robot(_ROBOT_PORT):
        print("OK — robot responds to Modbus ping")
    else:
        print("WARNING — no valid Modbus response (robot may not be ready yet)")
        print("  Continuing anyway; the bridge will work once the robot is live.")

    # ── 4. Open robot serial ───────────────────────────────────────────────
    try:
        ser_robot = serial.Serial(_ROBOT_PORT, BAUD, parity=PARITY, timeout=0.05)
    except serial.SerialException as e:
        print(f"\n[ERROR] Cannot open {_ROBOT_PORT}: {e}")
        print("  Is another program (STM32CubeIDE, Putty, test_robot.py) holding the port?")
        sys.exit(1)

    # ── 5. Locate com0com pair for main.exe passthrough ────────────────────
    ser_com10: Optional[serial.Serial] = None
    bridge_side, mainexe_side = _find_com0com_pair(exclude=_ROBOT_PORT)

    if bridge_side:
        try:
            ser_com10 = serial.Serial(bridge_side, BAUD, parity=PARITY, timeout=0.05)
        except serial.SerialException:
            ser_com10 = None

    # ── 6. Startup summary ─────────────────────────────────────────────────
    print()
    print("=" * 60)
    print(f"  Robot (STM32)  : {_ROBOT_PORT}  [OK]")
    if ser_com10:
        print(f"  Bridge side    : {bridge_side}")
        print(f"  *** main.exe   : configure to use {mainexe_side} ***")
    else:
        print("  [WARN] com0com pair not found or busy — Modbus passthrough DISABLED")
        print("         Install com0com or close whatever holds the virtual ports.")
    print(f"  WebSocket      : ws://localhost:{WS_PORT}")
    print("=" * 60)
    print("\nPress Ctrl+C to stop.\n")

    # Launch I/O threads
    threading.Thread(
        target=_http_server_thread, daemon=True, name="http-server"
    ).start()
    threading.Thread(
        target=_robot_reader, args=(ser_robot,), daemon=True, name="robot-rx"
    ).start()
    threading.Thread(
        target=_robot_writer, args=(ser_robot,), daemon=True, name="robot-tx"
    ).start()
    threading.Thread(
        target=_heartbeat_thread, daemon=True, name="heartbeat"
    ).start()
    print(f"[hb] Heartbeat proxy started — sending HI every {HB_SEND_INTERVAL_S*1000:.0f} ms")

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

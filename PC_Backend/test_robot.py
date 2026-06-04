"""
test_robot.py — Homing / Jog / Auto diagnostic. No base system needed.

Usage:
    python test_robot.py          # auto-scan for robot
    python test_robot.py COM10    # explicit port

Install:  pip install pyserial
Stop base system before running.
"""

import sys, time, serial, serial.tools.list_ports
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────
PORT    = sys.argv[1] if len(sys.argv) > 1 else None
BAUD    = 230400
PARITY  = serial.PARITY_EVEN
ADDR    = 21

FSM_NAMES  = {0:'INIT', 1:'HOMING', 2:'IDLE', 3:'RUNNING', 4:'FAULT'}
MODE_NAMES = {0:'idle', 1:'jog',    2:'auto', 3:'point',   4:'test'}
TASK_NAMES = {0:'Idle', 1:'Homing', 2:'GoPick', 4:'GoPlace', 8:'GoPoint'}

# ── CRC + packet ──────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def _pkt(payload: bytes) -> bytes:
    c = _crc16(payload)
    return payload + bytes([c & 0xFF, c >> 8])

# ── low-level Modbus ──────────────────────────────────────────────────────
def _read_n(ser, n, timeout=0.5):
    """Read exactly n bytes, printing any interleaved $-telemetry lines."""
    buf = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        ch = ser.read(1)
        if not ch:
            continue
        if ch == b'$' and not buf:
            line = '$' + ser.readline().decode('ascii', errors='ignore').strip()
            _tel(line)
            continue
        buf += ch
        if len(buf) >= n:
            return buf
    return None

def write_reg(ser, reg: int, val: int) -> bool:
    """FC06 write single register. val may be signed."""
    val &= 0xFFFF
    pkt = _pkt(bytes([ADDR, 0x06, reg >> 8, reg & 0xFF, val >> 8, val & 0xFF]))
    ser.write(pkt)
    ack = _read_n(ser, 8)
    return ack is not None and len(ack) == 8

def read_reg(ser, reg: int):
    """FC03 read single register. Returns int or None."""
    pkt = _pkt(bytes([ADDR, 0x03, reg >> 8, reg & 0xFF, 0x00, 0x01]))
    ser.write(pkt)
    r = _read_n(ser, 7)
    if r and len(r) == 7 and r[1] == 0x03:
        return (r[3] << 8) | r[4]
    return None

def read_regs(ser, start: int, count: int):
    """FC03 read multiple registers. Returns list or None."""
    pkt = _pkt(bytes([ADDR, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF]))
    ser.write(pkt)
    exp = 3 + count * 2 + 2
    r = _read_n(ser, exp)
    if r and len(r) == exp and r[1] == 0x03:
        return [(r[3+i*2] << 8) | r[3+i*2+1] for i in range(count)]
    return None

def s16(raw) -> int:
    """Raw uint16 → signed int16."""
    if raw is None: return 0
    return raw if raw < 32768 else raw - 65536

# ── telemetry decoder ─────────────────────────────────────────────────────
_HOM_LABELS = {
    'EA1':  ('edgeA',  'sweep_amp'),
    'OVS':  ('edgeA',  'ovs_pos'),
    'EB1':  ('edgeA',  'edgeB'),
    'CTR':  ('center', 'width'),
    'CEND': ('center', 'err'),
    'ZERO': ('center', 'park'),
}

def _tel(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    if line.startswith('$ST'):
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                fsm   = int(parts[1])
                mode  = int(parts[2])
                fault = int(parts[3])
                tag = f"STATE={FSM_NAMES.get(fsm,fsm)} mode={MODE_NAMES.get(mode,mode)}"
                if fault:
                    tag += f" FAULT_CODE={fault}"
                print(f"  [{ts}] {tag}")
                return
            except ValueError:
                pass
    elif line.startswith('$HOM'):
        parts = line.split(',')
        if len(parts) >= 6:
            tag = parts[1]
            try:
                v1  = int(parts[2]) / 100
                v2  = int(parts[3]) / 100
                pos = int(parts[5]) / 100
                n1, n2 = _HOM_LABELS.get(tag, ('v1', 'v2'))
                print(f"  [{ts}] $HOM {tag:<4}  {n1}={v1:.2f}°  {n2}={v2:.2f}°  pos={pos:.2f}°")
                return
            except (ValueError, IndexError):
                pass
    print(f"  [{ts}] {line}")

def drain(ser, secs=0.15):
    """Drain + print pending telemetry for 'secs' seconds."""
    t0 = ser.timeout
    ser.timeout = 0.05
    deadline = time.time() + secs
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            txt = raw.decode('ascii', errors='ignore').strip()
            if txt.startswith('$'):
                _tel(txt)
    ser.timeout = t0

# ── status helpers ────────────────────────────────────────────────────────
def print_status(ser):
    drain(ser, 0.2)
    task_raw = read_reg(ser, 0x27)
    pos_raw  = read_reg(ser, 0x28)
    vel_raw  = read_reg(ser, 0x29)
    sensors  = read_reg(ser, 0x26)
    emerg    = read_reg(ser, 0x31)
    pos   = s16(pos_raw) / 10.0
    vel   = s16(vel_raw) / 10.0
    task  = TASK_NAMES.get(task_raw, f'0x{task_raw:04X}') if task_raw is not None else '?'
    estop = bool(emerg & 1) if emerg is not None else '?'
    fc    = (emerg >> 8) if emerg is not None else '?'
    prox  = bool(sensors & 0x08) if sensors is not None else '?'
    print(f"  pos={pos:.1f}°  vel={vel:.1f}°/s  task={task}  prox={prox}  estop={estop}  fault_code={fc}")

def wait_idle(ser, timeout=20.0, label='') -> bool:
    """Poll task reg until 0. Print progress. Return True if reached."""
    deadline = time.time() + timeout
    last_pos = None
    while time.time() < deadline:
        drain(ser, 0.05)
        task_raw = read_reg(ser, 0x27)
        pos_raw  = read_reg(ser, 0x28)
        emerg    = read_reg(ser, 0x31)

        # fault check via emergency reg upper byte
        if emerg is not None and (emerg >> 8) != 0:
            fc = emerg >> 8
            pos = s16(pos_raw) / 10.0
            print(f"  ✗ FAULT code={fc}  pos={pos:.1f}°")
            return False

        if task_raw == 0:
            pos = s16(pos_raw) / 10.0
            print(f"  → Idle  pos={pos:.1f}°")
            return True

        pos = s16(pos_raw) / 10.0
        if pos != last_pos:
            task = TASK_NAMES.get(task_raw, f'task={task_raw}')
            print(f"  ... {task:<8}  pos={pos:.1f}°")
            last_pos = pos
        time.sleep(0.25)

    print(f"  ✗ Timeout ({timeout:.0f}s)")
    return False

# ── tests ─────────────────────────────────────────────────────────────────
def test_homing(ser):
    print("\n══ HOMING TEST ══")
    drain(ser, 0.5)
    print_status(ser)

    input("\nPress ENTER to send Home command...")
    t0 = time.time()
    write_reg(ser, 0x01, 1)
    print("  >> mode=1 (Home) sent\n")

    # Capture $HOM events until ZERO or FAULT/timeout
    old_to = ser.timeout
    ser.timeout = 0.5
    deadline = t0 + 120.0
    ok = False

    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('ascii', errors='ignore').strip()
        if line.startswith('$HOM') or line.startswith('$ST'):
            _tel(line)
            if 'ZERO' in line:
                print(f"\n  ✓ Homing complete  ({time.time()-t0:.1f}s)")
                ok = True
                break
            if line.startswith('$ST') and ',4,' in line:
                print(f"\n  ✗ FAULT during homing")
                break

    ser.timeout = old_to
    if not ok and time.time() >= deadline:
        print("  ✗ Timeout (120s) — no ZERO event")
    return ok


def test_jog(ser, step_deg: int = 15):
    print(f"\n══ JOG TEST ({step_deg:+d}°) ══")
    drain(ser, 0.3)

    pos_init = s16(read_reg(ser, 0x28)) / 10.0
    print(f"  Initial pos: {pos_init:.1f}°")

    # Write step BEFORE mode (firmware needs step nonzero when mode is read)
    step_wire = step_deg & 0xFFFF
    write_reg(ser, 0x05, step_wire)
    time.sleep(0.05)
    write_reg(ser, 0x01, 2)           # mode = Jog
    print(f"  >> step={step_deg}°, mode=2 (Jog) sent")

    ok = wait_idle(ser, timeout=15.0)

    pos_final = s16(read_reg(ser, 0x28)) / 10.0
    moved = pos_final - pos_init
    print(f"  Final pos: {pos_final:.1f}°  (moved {moved:+.1f}°)")

    if abs(moved) > 1.0:
        print(f"  ✓ Jog success")
        return True
    else:
        print(f"  ✗ Motor barely moved — check step register or motor hardware")
        return False


def test_auto(ser, pairs=((1, 2),)):
    """
    pairs: list of (pick_index, place_index), indices 1-10 (match g_idx_deg table).
    Default: 1 pair — pick at index 1 (0°), place at index 2 (36°).
    """
    n = min(len(pairs), 8)
    print(f"\n══ AUTO TEST ({n} pair(s)) ══")
    drain(ser, 0.3)
    print_status(ser)

    # Write slots: reg 0x12 = slot0 (pick0), 0x13 = slot1 (place0), ...
    for i, (pick, place) in enumerate(pairs[:n]):
        write_reg(ser, 0x12 + i*2,     pick  & 0xFFFF)
        write_reg(ser, 0x12 + i*2 + 1, place & 0xFFFF)

    write_reg(ser, 0x22, n)   # pair count — must write BEFORE mode
    write_reg(ser, 0x04, 0)   # gripper disabled
    time.sleep(0.05)
    write_reg(ser, 0x01, 4)   # mode = Auto
    print(f"  >> pairs={n}, mode=4 (Auto) sent\n")

    ok = wait_idle(ser, timeout=60.0 * n)
    if ok:
        print("  ✓ Auto sequence complete")
    else:
        print("  ✗ Auto failed")
    return ok


def soft_stop(ser):
    write_reg(ser, 0x25, 1)
    time.sleep(0.05)
    write_reg(ser, 0x25, 0)
    print("  Soft stop sent")


# ── port scan ─────────────────────────────────────────────────────────────
def find_port():
    for p in sorted(serial.tools.list_ports.comports()):
        try:
            with serial.Serial(p.device, BAUD, parity=PARITY, timeout=0.2) as s:
                s.reset_input_buffer()
                s.write(_pkt(bytes([ADDR, 0x03, 0x00, 0x27, 0x00, 0x01])))
                r = s.read(7)
                if len(r) == 7 and r[1] == 0x03:
                    return p.device
        except Exception:
            pass
    return None


# ── main menu ─────────────────────────────────────────────────────────────
def main():
    print("Available ports:")
    for p in sorted(serial.tools.list_ports.comports()):
        print(f"  {p.device:<10} {p.description}")

    port = PORT
    if port is None:
        print("\nScanning for robot...")
        port = find_port()
        if port is None:
            print("Robot not found. Ensure base system is stopped and robot is powered.")
            sys.exit(1)
        print(f"Found on {port}")

    print(f"\nConnecting {port} @ {BAUD} 8E1 ...")

    with serial.Serial(port, BAUD, parity=PARITY, timeout=0.5) as ser:
        ser.reset_input_buffer()

        # Sanity ping
        task = read_reg(ser, 0x27)
        if task is None:
            print("No Modbus response. Check port / ensure base system is closed.")
            sys.exit(1)

        print("Robot online.")
        print_status(ser)
        drain(ser, 1.0)   # show any $ST lines already pending

        MENU = """
Commands:
  h       Homing test
  j       Jog +15° (CCW)
  J       Jog -15° (CW)
  jN      Jog N degrees  (e.g. j30, j-20)
  a       Auto test — 1 pair (index 1→2)
  s       Soft stop
  r       Read status
  q       Quit
"""
        print(MENU)

        while True:
            try:
                cmd = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not cmd:
                continue
            elif cmd == 'q':
                break
            elif cmd == 'h':
                test_homing(ser)
            elif cmd == 'j':
                test_jog(ser, 15)
            elif cmd == 'J':
                test_jog(ser, -15)
            elif cmd.startswith('j') and len(cmd) > 1:
                try:
                    test_jog(ser, int(cmd[1:]))
                except ValueError:
                    print("  Bad step. Use e.g. j30 or j-20")
            elif cmd == 'a':
                test_auto(ser, pairs=[(1, 2)])
            elif cmd == 's':
                soft_stop(ser)
            elif cmd == 'r':
                print_status(ser)
                drain(ser, 0.5)
            else:
                print(MENU)


if __name__ == '__main__':
    main()

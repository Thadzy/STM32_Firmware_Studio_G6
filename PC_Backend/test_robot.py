"""
test_robot.py — Homing / Jog / Auto diagnostic. No base system needed.

Usage:
    python test_robot.py          # auto-scan for robot
    python test_robot.py COM10    # explicit port

Install:  pip install pyserial
Stop base system before running.
"""

import sys, time, serial, serial.tools.list_ports
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.stderr.reconfigure(encoding='utf-8', errors='replace')
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────
PORT    = sys.argv[1] if len(sys.argv) > 1 else None
BAUD    = 230400
PARITY  = serial.PARITY_EVEN
ADDR    = 21
HB_VAL  = 18537   # "HI" — PC reply to robot's 22881 "YA" heartbeat

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

_quiet = True    # suppress $T telemetry spam during interactive monitors

def _tel(line: str):
    if _quiet and line.startswith('$T,'):
        return
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

# ── heartbeat ─────────────────────────────────────────────────────────────
_hb_last = 0.0

def beat(ser):
    """Write HI heartbeat if >1 s elapsed. Call inside any polling loop."""
    global _hb_last
    if time.time() - _hb_last >= 0.5:   # ~2 Hz; firmware times out at 2 s
        write_reg(ser, 0x00, HB_VAL)
        _hb_last = time.time()

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
    reed_up   = bool(sensors & 0x01) if sensors is not None else '?'
    reed_down = bool(sensors & 0x02) if sensors is not None else '?'
    reed_close= bool(sensors & 0x04) if sensors is not None else '?'
    print(f"  pos={pos:.1f}°  vel={vel:.1f}°/s  task={task}")
    print(f"  prox={prox}  estop={estop}  fault_code={fc}")
    print(f"  reed: up={reed_up}  down={reed_down}  closed={reed_close}")

def wait_idle(ser, timeout=20.0, label='') -> bool:
    """Poll task reg until 0. Print progress. Return True if reached."""
    deadline = time.time() + timeout
    last_pos = None
    while time.time() < deadline:
        beat(ser)
        drain(ser, 0.05)
        task_raw = read_reg(ser, 0x27)
        pos_raw  = read_reg(ser, 0x28)
        emerg    = read_reg(ser, 0x31)

        # fault check — ignore 0x20 (PC link lost: cleared by heartbeat above)
        if emerg is not None and (emerg >> 8) not in (0, 0x20):
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
        beat(ser)
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

    # Wait for the firmware to actually START the sequence (task 0x27 != 0)
    # before monitoring. Otherwise wait_idle reads the still-0 task on the very
    # first poll, declares "Idle" instantly, and returns to the blocking input()
    # prompt — where beat() stops firing, the 2 s heartbeat lapses, and the
    # robot soft-stops mid-run with fault_code 0x20 after only the first move.
    t0 = time.time()
    started = False
    while time.time() - t0 < 3.0:
        beat(ser)
        drain(ser, 0.05)
        if read_reg(ser, 0x27):
            started = True
            break
        time.sleep(0.1)
    if not started:
        print("  ✗ Auto did not start (task stayed Idle). Is the robot homed and in IDLE?")
        return False

    ok = wait_idle(ser, timeout=60.0 * n)
    if ok:
        print("  ✓ Auto sequence complete")
    else:
        print("  ✗ Auto failed")
    return ok


# ── collision-aware random auto sequencer ──────────────────────────────────
# Firmware fact (verified in app_main.c resolve_target + motor_controller.c):
#   In Auto mode the arm moves to the ABSOLUTE angle  index * 5°  (sign of the
#   slot is ignored) and the S-curve drives there LINEARLY — no wrap-around /
#   shortest-path. So a pick→place pair always sweeps the holes numerically
#   BETWEEN pick and place; that arc is fixed and cannot be reversed from the
#   PC. The only lever we (or the Base System) have over collisions is the
#   ORDER in which pairs run, because that changes which holes still hold a rod
#   at the moment of transport. Everything below uses ONLY the documented
#   registers 0x12–0x21 (slots) + 0x22 (count), so it is reproducible by the
#   Base System with no firmware change.

def _between(a: int, b: int) -> set:
    """Hole indices strictly swept when moving from index a to index b."""
    lo, hi = min(abs(a), abs(b)), max(abs(a), abs(b))
    return set(range(lo + 1, hi))

def plan_safe_order(pairs, rng, attempts=4000):
    """Return a random collision-safe ordering of (pick, place) pairs, or None.

    Model: every pick hole starts occupied (a rod is in it). To run a pair we
    (1) lift the rod at `pick` (that hole becomes empty during transport),
    (2) sweep pick→place — every hole still occupied on that arc is a collision,
    (3) the destination `place` must itself be empty, then becomes occupied.
    Backtracking search with randomized branch order yields a *random* valid order.
    """
    n = len(pairs)
    start_occupied = frozenset(abs(p) for p, _ in pairs)

    def search(remaining, occupied, acc):
        if not remaining:
            return acc
        order = list(remaining)
        rng.shuffle(order)
        for i in order:
            pick, place = abs(pairs[i][0]), abs(pairs[i][1])
            occ_transport = occupied - {pick}          # rod lifted off pick
            if place in occ_transport:                 # destination must be free
                continue
            if _between(pick, place) & occ_transport:  # arc must be clear
                continue
            res = search(remaining - {i},
                         occ_transport | {place},
                         acc + [i])
            if res is not None:
                return res
        return None

    # A few independent randomized restarts so the chosen order really varies.
    for _ in range(max(1, attempts // max(1, n))):
        res = search(frozenset(range(n)), set(start_occupied), [])
        if res is not None:
            return [pairs[i] for i in res]
    return None


def test_auto_random(ser, n_pairs=3, max_hole=10, gripper=False, seed=None):
    """Generate random pick/place pairs, order them collision-safe, run Auto.

    Uses the same write path as test_auto (regs 0x12–0x21, 0x22) so the
    behavior is identical to what the Base System produces."""
    import random
    rng = random.Random(seed)
    n_pairs = max(1, min(n_pairs, 8))
    max_hole = max(2, min(max_hole, 72))

    if max_hole < 2 * n_pairs:
        print(f"  Need at least {2*n_pairs} holes for {n_pairs} distinct pick+place pairs "
              f"(max_hole={max_hole}). Reduce pairs or raise max_hole.")
        return False

    print(f"\n══ AUTO RANDOM ({n_pairs} pair(s), holes 1–{max_hole}, "
          f"gripper={'on' if gripper else 'off'}) ══")

    # Re-roll random pair sets until one has a collision-safe order. Many dense
    # random pairings are physically infeasible (a carried rod would always
    # sweep an occupied hole), so we keep drawing fresh pairs rather than fail.
    pairs = ordered = None
    REROLLS = 2000
    for attempt in range(REROLLS):
        holes = rng.sample(range(1, max_hole + 1), 2 * n_pairs)  # distinct holes
        cand = [(holes[2 * i], holes[2 * i + 1]) for i in range(n_pairs)]
        order = plan_safe_order(cand, rng)
        if order is not None:
            pairs, ordered = cand, order
            if attempt:
                print(f"  (found a collision-safe set after {attempt+1} re-rolls)")
            break

    if ordered is None:
        print(f"  ✗ No collision-safe random set found in {REROLLS} tries for "
              f"{n_pairs} pairs in holes 1–{max_hole}.")
        print("    Try fewer pairs or a larger hole range (raise max_hole).")
        return False

    print("  Generated pairs (pick → place):")
    for p, q in pairs:
        print(f"    {p:>2} ({p*5}°) → {q:>2} ({q*5}°)")

    print("\n  Collision-safe run order:")
    for k, (p, q) in enumerate(ordered, 1):
        swept = sorted(_between(p, q))
        swept_s = f"  sweeps {swept}" if swept else "  (adjacent, no holes between)"
        print(f"    {k}. pick {p:>2} → place {q:>2}{swept_s}")

    if gripper:
        write_reg(ser, 0x04, 1)
    ok = test_auto(ser, pairs=ordered)
    if gripper:
        write_reg(ser, 0x04, 0)
    return ok


def wait_grip(ser, timeout=12.0) -> bool:
    """Poll reg 0x03 until 0 (sequence complete) or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        drain(ser, 0.05)
        sensors = read_reg(ser, 0x26)
        val     = read_reg(ser, 0x03)
        reed_up   = bool(sensors & 0x01) if sensors is not None else '?'
        reed_down = bool(sensors & 0x02) if sensors is not None else '?'
        reed_close= bool(sensors & 0x04) if sensors is not None else '?'
        print(f"  ... grip_reg={val}  up={reed_up}  down={reed_down}  closed={reed_close}")
        if val == 0:
            return True
        time.sleep(0.4)
    print(f"  x Timeout ({timeout:.0f}s)")
    return False


def test_grip(ser, pick: bool):
    label = 'PICK' if pick else 'PLACE'
    print(f"\n══ GRIPPER {label} TEST ══")
    drain(ser, 0.3)

    sensors = read_reg(ser, 0x26)
    reed_up   = bool(sensors & 0x01) if sensors is not None else '?'
    reed_down = bool(sensors & 0x02) if sensors is not None else '?'
    reed_close= bool(sensors & 0x04) if sensors is not None else '?'
    print(f"  Before: up={reed_up}  down={reed_down}  closed={reed_close}")

    cmd = 1 if pick else 2
    write_reg(ser, 0x03, cmd)
    print(f"  >> reg 0x03 = {cmd} ({label}) sent")

    ok = wait_grip(ser)

    sensors = read_reg(ser, 0x26)
    reed_up   = bool(sensors & 0x01) if sensors is not None else '?'
    reed_down = bool(sensors & 0x02) if sensors is not None else '?'
    reed_close= bool(sensors & 0x04) if sensors is not None else '?'
    print(f"  After:  up={reed_up}  down={reed_down}  closed={reed_close}")
    print(f"  {'OK' if ok else 'TIMEOUT'} — {label} sequence {'complete' if ok else 'did not finish'}")
    return ok


def soft_stop(ser):
    write_reg(ser, 0x25, 1)
    time.sleep(0.05)
    write_reg(ser, 0x25, 0)
    print("  Soft stop sent")


def monitor_prox(ser):
    """Live proximity/reed monitor. Move the arm by hand over the sensor and
    watch the prox bit. Ctrl+C to stop. Pure FC03 polling, no motion."""
    global _quiet
    print("\n══ PROXIMITY LIVE MONITOR ══")
    print("  Move the arm slowly across the sensor. Watch prox.")
    print("  Also check the sensor's own indicator LED. Ctrl+C to stop.\n")
    _quiet = True
    last = None
    try:
        while True:
            sensors = read_reg(ser, 0x26)
            pos_raw = read_reg(ser, 0x28)
            if sensors is None:
                print("  (no response)")
                time.sleep(0.2)
                continue
            prox  = bool(sensors & 0x08)
            r_up  = bool(sensors & 0x01)
            r_dn  = bool(sensors & 0x02)
            r_cl  = bool(sensors & 0x04)
            pos   = s16(pos_raw) / 10.0
            mark = "  <<< PROX!" if prox else ""
            line = f"prox={int(prox)}  up={int(r_up)} dn={int(r_dn)} cl={int(r_cl)}  pos={pos:.1f}deg{mark}"
            if line != last:                    # only print on change to reduce noise
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"  [{ts}] {line}")
                last = line
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n  Monitor stopped.")
    finally:
        _quiet = False


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
  gp      Gripper Pick sequence (down→close→up)
  gl      Gripper Place sequence (down→open→up)
  a       Auto test — 1 pair, no gripper
  ag      Auto test — 1 pair, gripper enabled
  ar      Auto RANDOM — random pairs, collision-safe order, no gripper
  arg     Auto RANDOM — random pairs, collision-safe order, gripper enabled
  arN     Auto RANDOM with N pairs (e.g. ar4, arg2)
  s       Soft stop
  r       Read status
  p       Proximity live monitor (move arm by hand, watch prox bit)
  oN      Set homing offset to N degrees  (e.g. o3, o-3)  used after next home
  sh      Set home HERE — zero encoder at current position immediately (pos → 0°)
  q       Quit
"""
        print(MENU)

        while True:
            beat(ser)
            try:
                cmd = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not cmd:
                beat(ser)
                continue
            elif cmd == 'q':
                break
            elif cmd == 'h':
                test_homing(ser)
            elif cmd == 'j':
                test_jog(ser, 10)
            elif cmd == 'J':
                test_jog(ser, -10)
            elif cmd.startswith('j') and len(cmd) > 1:
                try:
                    test_jog(ser, int(cmd[1:]))
                except ValueError:
                    print("  Bad step. Use e.g. j30 or j-20")
            elif cmd == 'gp':
                test_grip(ser, pick=True)
            elif cmd == 'gl':
                test_grip(ser, pick=False)
            elif cmd.startswith('arg'):
                try:
                    n = int(cmd[3:]) if len(cmd) > 3 else 3
                    test_auto_random(ser, n_pairs=n, gripper=True)
                except ValueError:
                    print("  Bad count. Use e.g. arg or arg4")
            elif cmd.startswith('ar'):
                try:
                    n = int(cmd[2:]) if len(cmd) > 2 else 3
                    test_auto_random(ser, n_pairs=n, gripper=False)
                except ValueError:
                    print("  Bad count. Use e.g. ar or ar4")
            elif cmd == 'a':
                test_auto(ser, pairs=[(1, 2)])
            elif cmd == 'ag':
                write_reg(ser, 0x04, 1)   # gripper enable
                test_auto(ser, pairs=[(1, 2)])
                write_reg(ser, 0x04, 0)   # disable after
            elif cmd == 's':
                soft_stop(ser)
            elif cmd == 'r':
                print_status(ser)
                drain(ser, 0.5)
            elif cmd == 'p':
                monitor_prox(ser)
            elif cmd == 'sh':
                write_reg(ser, 0x01, 8)   # SetHome: zero encoder at current pos
                time.sleep(0.1)
                pos_raw = read_reg(ser, 0x28)
                print(f"  Home set. pos={s16(pos_raw)/10.0:.1f}deg (should be 0.0)")
            elif cmd.startswith('o') and len(cmd) > 1:
                try:
                    offset_deg = int(cmd[1:])
                    write_reg(ser, 0x32, offset_deg & 0xFFFF)
                    print(f"  Home offset set to {offset_deg} deg (reg 0x32)")
                except ValueError:
                    print("  Bad offset. Use e.g. o3 or o-3")
            else:
                print(MENU)


if __name__ == '__main__':
    main()

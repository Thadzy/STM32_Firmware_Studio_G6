"""
homing_log.py  —  Trigger homing and print the log, no base system needed.

Usage:
    python homing_log.py COM10       # replace with your actual port

Install once:
    pip install pyserial

IMPORTANT: Stop the base system before running this.
"""

import sys
import time
import serial
import serial.tools.list_ports
from datetime import datetime

PORT    = sys.argv[1] if len(sys.argv) > 1 else None
BAUD    = 230400
PARITY  = serial.PARITY_EVEN
ADDR    = 21        # Modbus slave address
TIMEOUT = 120.0     # seconds to wait for homing to complete

# ---------------------------------------------------------------------------
# Modbus RTU helpers
# ---------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def make_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])

def write_register(ser: serial.Serial, reg: int, value: int):
    """Modbus FC06 — write single register. Read exactly 8 ACK bytes instead
    of reset_input_buffer so early telemetry is not accidentally discarded."""
    pkt = make_packet(bytes([ADDR, 0x06, reg >> 8, reg & 0xFF,
                              value >> 8, value & 0xFF]))
    ser.write(pkt)
    ser.read(8)               # consume FC06 echo ACK (exactly 8 bytes)

def read_register(ser: serial.Serial, reg: int) -> int | None:
    """Modbus FC03 — read single register. Returns int or None on error."""
    pkt = make_packet(bytes([ADDR, 0x03, reg >> 8, reg & 0xFF, 0x00, 0x01]))
    ser.write(pkt)
    time.sleep(0.05)
    resp = ser.read(7)        # addr(1)+fc(1)+len(1)+data(2)+crc(2)
    if len(resp) == 7 and resp[1] == 0x03:
        return (resp[3] << 8) | resp[4]
    return None

# ---------------------------------------------------------------------------
# Decode $HOM lines (values are integer×100 because newlib-nano has no %f)
# Format: $HOM,TAG,v1x100,v2x100,POS,posx100
# ---------------------------------------------------------------------------

TAG_LABELS = {
    "EA1":  ("edgeA",  "amp"    ),
    "OVS":  ("edgeA",  "ovs_end"),
    "EB1":  ("edgeA",  "edgeB"  ),
    "CTR":  ("center", "width"  ),
    "CEND": ("center", "err"    ),
    "ZERO": ("center", "park"   ),
}

def decode_hom(line: str) -> str:
    parts = line.split(",")
    if len(parts) < 6:
        return line
    tag = parts[1]
    try:
        v1  = int(parts[2]) / 100.0
        v2  = int(parts[3]) / 100.0
        pos = int(parts[5]) / 100.0
    except (ValueError, IndexError):
        return line
    n1, n2 = TAG_LABELS.get(tag, ("v1", "v2"))
    return f"$HOM,{tag}  {n1}={v1:.2f}°  {n2}={v2:.2f}°  pos={pos:.2f}°"

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  (no COM ports found)")
    for p in sorted(ports):
        print(f"  {p.device:<8}  {p.description}")

def find_robot_port() -> str | None:
    """Try every available port and return the first that answers Modbus."""
    ports = serial.tools.list_ports.comports()
    for p in sorted(ports):
        try:
            with serial.Serial(p.device, BAUD, parity=PARITY, timeout=0.15) as ser:
                ser.reset_input_buffer()
                val = read_register(ser, 0x27)
                if val is not None:
                    return p.device
        except serial.SerialException:
            pass
    return None

# ---------------------------------------------------------------------------
# Port selection
# ---------------------------------------------------------------------------

print("Available COM ports:")
list_ports()
print()

if PORT is None:
    print("No port given — scanning for robot...")
    PORT = find_robot_port()
    if PORT is None:
        print("Robot not found on any port.")
        print("Make sure the base system is closed and the robot is powered on.")
        sys.exit(1)
    print(f"Found robot on {PORT}")

print(f"\nOpening {PORT} at {BAUD} baud")
print("-" * 50)

try:
    with serial.Serial(PORT, BAUD, parity=PARITY, timeout=0.5) as ser:
        ser.reset_input_buffer()

        # Check robot state before sending home
        state = read_register(ser, 0x27)
        if state is not None:
            print(f"Robot task register (0x27) = 0x{state:04X}")
        else:
            print(f"No response on {PORT} — check that the base system is closed.")
            sys.exit(1)

        input("\nPress ENTER to trigger homing... ")
        t_start = time.time()
        write_register(ser, 0x01, 1)
        print("  >> Home command sent\n")

        events = []
        deadline = t_start + TIMEOUT

        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("$HOM"):
                continue

            now     = time.time()
            elapsed = now - t_start
            ts      = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # Decode integer×100 values back to degrees
            decoded = decode_hom(line)

            # Delta from previous event
            if events:
                delta     = now - events[-1][0]
                delta_str = f"  (+{delta:.2f}s)"
            else:
                delta_str = ""

            events.append((now, line))
            print(f"[{ts}  T+{elapsed:6.2f}s]  {decoded}{delta_str}")

            if "ZERO" in line:
                break
        else:
            print("\nTimeout — no ZERO event received.")
            sys.exit(1)

        # Summary
        if events:
            print("\n--- Timing summary (elapsed since home trigger) ---")
            labels = {
                "EA1":  "Edge A found",
                "OVS":  "Overshoot done — reversed",
                "EB1":  "Edge B found from other side",
                "CTR":  "Center calculated",
                "CEND": "GO_CENTER done",
                "ZERO": "Encoder zeroed",
            }
            for t, line in events:
                elapsed = t - t_start
                tag  = line.split(",")[1] if "," in line else "?"
                desc = labels.get(tag, "")
                print(f"  T+{elapsed:6.2f}s  [{tag}]  {desc}")
            print(f"  ------")
            print(f"  T+{events[-1][0] - t_start:6.2f}s  total")

        print("\nDone.")

except serial.SerialException as e:
    print(f"Cannot open {PORT}: {e}")
    print("Make sure the base system is closed and the port is correct.")
except KeyboardInterrupt:
    print("\nStopped.")

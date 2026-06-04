"""
diag_com10.py — raw diagnostic for COM10 at 230400 baud.
Sends a Modbus FC03 read on register 0x27 (slave 21) and prints
everything received for 1 second, then prints raw hex.
"""
import serial, time

BAUD = 230400
PORT = "COM10"
ADDR = 21

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def make_pkt(payload):
    c = crc16(payload)
    return payload + bytes([c & 0xFF, c >> 8])

print(f"Opening {PORT} @ {BAUD}...")
with serial.Serial(PORT, BAUD, parity=serial.PARITY_EVEN, timeout=1.0) as ser:
    ser.reset_input_buffer()

    # Send FC03 read of register 0x27 (1 register)
    pkt = make_pkt(bytes([ADDR, 0x03, 0x00, 0x27, 0x00, 0x01]))
    print(f"TX: {pkt.hex(' ')}")
    ser.write(pkt)

    # Wait and collect everything
    time.sleep(1.0)
    rx = ser.read(ser.in_waiting or 1)

print(f"RX ({len(rx)} bytes): {rx.hex(' ') if rx else '(nothing)'}")
if rx:
    try:
        print(f"As ASCII: {rx.decode('ascii', errors='replace')!r}")
    except Exception:
        pass

print()
print("--- Also listening for 2s of free-running output ---")
with serial.Serial(PORT, BAUD, parity=serial.PARITY_EVEN, timeout=0.1) as ser:
    ser.reset_input_buffer()
    deadline = time.time() + 2.0
    lines = []
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            lines.append(raw)
            print(f"  {raw.hex(' ')}  |  {raw.decode('ascii', errors='replace')!r}")
    if not lines:
        print("  (no data received)")

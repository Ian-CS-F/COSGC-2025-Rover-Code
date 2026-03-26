"""
Test Protocol: Serial Communications (Pi <-> Arduino)
Tests the READY/ACK handshake and command/response cycle exactly as used in main.py.
Requires the Arduino to be connected and running test_comms.ino.
"""

import time
import serial

# ── Constants ─────────────────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyACM0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10

# ── Tests (all share one serial connection) ───────────────────────────────────
def test_port_opens(ser):
    print(f"TEST: Open serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    print("  PASS")
    return True

def test_arduino_sends_ready(ser):
    print("TEST: Arduino sends READY within 10 s...")
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            print("  PASS — received READY")
            return True
    print("  FAIL — READY not received within timeout")
    return False

def test_handshake_completes(ser):
    print("TEST: Full READY/ACK handshake...")
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            print("  PASS — handshake complete")
            return True
    print("  FAIL — READY not received")
    return False

def test_query_distance(ser):
    print("TEST: QUERY command returns DIST:<cm> response...")
    ser.write(b"(3,0,0.000,0.000)\n")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            dist = float(line[5:])
            print(f"  Distance since last query: {dist:.1f} cm  →  PASS")
            return True
    print("  FAIL — no DIST: response received")
    return False

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # let Arduino reset after DTR toggle
        ser.reset_input_buffer()
    except Exception as e:
        print(f"ERROR: Could not open port — {e}")
        exit(1)

    tests = [
        test_port_opens,
        test_arduino_sends_ready,
        test_handshake_completes,
        test_query_distance,
    ]

    results = []
    for t in tests:
        results.append((t.__name__, t(ser)))

    ser.close()

    print("\n--- Comms Test Results ---")
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
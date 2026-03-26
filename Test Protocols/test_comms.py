"""
Test Protocol: Serial Communications (Pi <-> Arduino)
Tests the READY/ACK handshake and command/response cycle exactly as used in main.py.
Requires the Arduino to be connected and running rover_interpreter.ino.
"""

import time
import serial

# ── Constants from main.py ────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyACM0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10

# ── Tests ─────────────────────────────────────────────────────────────────────
def test_port_opens():
    """Serial port opens without error."""
    print(f"TEST: Open serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.close()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL — {e}")
        return False

def test_arduino_sends_ready():
    """Arduino sends 'READY' within the handshake timeout."""
    print("TEST: Arduino sends READY within 10 s...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line == "READY":
                print("  PASS — received READY")
                ser.close()
                return True
        print("  FAIL — READY not received within timeout")
        ser.close()
        return False
    except Exception as e:
        print(f"  FAIL — {e}")
        return False

def test_handshake_completes():
    """Full READY/ACK handshake completes exactly as init_serial() does it."""
    print("TEST: Full READY/ACK handshake...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line == "READY":
                ser.write(b"ACK\n")
                print("  PASS — handshake complete")
                ser.close()
                return True
        print("  FAIL — READY not received")
        ser.close()
        return False
    except Exception as e:
        print(f"  FAIL — {e}")
        return False

def test_query_distance():
    """
    After handshake, send a QUERY command (3,0,0.000,0.000) and verify
    the Arduino responds with 'DIST:<value>'.
    """
    print("TEST: QUERY command returns DIST:<cm> response...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # let Arduino reset after DTR toggle
        ser.reset_input_buffer()

        # Handshake first
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT
        ready = False
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line == "READY":
                ser.write(b"ACK\n")
                ready = True
                break

        if not ready:
            print("  FAIL — could not complete handshake")
            ser.close()
            return False

        # Send QUERY (system=3)
        time.sleep(0.5)
        ser.write(b"(3,0,0.000,0.000)\n")

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("DIST:"):
                dist = float(line[5:])
                print(f"  Distance since last query: {dist:.1f} cm  →  PASS")
                ser.close()
                return True

        print("  FAIL — no DIST: response received")
        ser.close()
        return False
    except Exception as e:
        print(f"  FAIL — {e}")
        return False

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    tests = [
        test_port_opens,
        test_arduino_sends_ready,
        test_handshake_completes,
        test_query_distance,
    ]
    results = []
    for t in tests:
        results.append((t.__name__, t()))
        time.sleep(1.0)  # allow Arduino to reset between tests

    print("\n--- Comms Test Results ---")
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")

"""
Test Protocol: Motors + Ultrasonic Sensors
Tests motor commands and reads DONE/CLIFF responses exactly as main.py does.
The ultrasonics live on the Arduino — cliff detection arrives as 'CLIFF:<cm>'
over serial, so both are verified here together.

Place the rover on a stand (wheels off ground) for the motor spin tests.
For ultrasonic tests, leave it on the ground and lift the front to simulate a cliff.

Requires Arduino running rover_interpreter.ino.
"""

import time
import serial

# ── Constants from main.py ────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyUSB0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10
MOTOR_DONE_TIMEOUT = 30.0

SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT, RIGHT, UP, DOWN = 0, 1, 2, 3

# ── Helper ────────────────────────────────────────────────────────────────────
def _open_and_handshake():
    """Open serial and complete the READY/ACK handshake. Returns ser or None."""
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            return ser
    ser.close()
    return None

def send_command(ser, system, direction, amount, speed):
    packet = f"({system},{direction},{amount:.3f},{speed:.3f})\n"
    ser.write(packet.encode())

def wait_for_done(ser, timeout=MOTOR_DONE_TIMEOUT):
    """Wait for DONE:<left_cm>,<right_cm>,<left_ratio>,<right_ratio> and return
    (left_cm, right_cm, left_ratio, right_ratio), or None on timeout.
    CLIFF messages are ignored during testing."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DONE:"):
            parts = line[5:].split(",")
            left_cm    = float(parts[0])
            right_cm   = float(parts[1]) if len(parts) > 1 else left_cm
            left_ratio = float(parts[2]) if len(parts) > 2 else 1.0
            right_ratio= float(parts[3]) if len(parts) > 3 else 1.0
            return left_cm, right_cm, left_ratio, right_ratio
    return None

# ── Tests ─────────────────────────────────────────────────────────────────────
def test_drive_forward():
    """
    Send (1, UP, 10.0, 0.7) — drive 10 cm forward — and confirm DONE is received.
    Rover should be on a stand so wheels spin freely.
    """
    print("TEST: Drive forward 10 cm (wheels off ground)...")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    send_command(ser, MOTOR, UP, 10.0, 0.7)
    result = wait_for_done(ser)
    ser.close()
    if result is None:
        print("  FAIL — no DONE received within timeout")
        return False
    left_cm, right_cm, left_ratio, right_ratio = result
    discrepancy = abs(left_cm - right_cm)
    passed = left_cm > 0.0 and right_cm > 0.0
    print(f"  Left: {left_cm:.1f} cm (ratio {left_ratio:.3f})  Right: {right_cm:.1f} cm (ratio {right_ratio:.3f})  "
          f"Discrepancy: {discrepancy:.1f} cm  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_drive_reverse():
    """Send (1, DOWN, 10.0, 0.7) and confirm DONE is received."""
    print("TEST: Drive reverse 10 cm (wheels off ground)...")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    send_command(ser, MOTOR, DOWN, 10.0, 0.7)
    result = wait_for_done(ser)
    ser.close()
    if result is None:
        print("  FAIL — no DONE received within timeout")
        return False
    left_cm, right_cm, left_ratio, right_ratio = result
    discrepancy = abs(left_cm - right_cm)
    passed = left_cm > 0.0 and right_cm > 0.0
    print(f"  Left: {left_cm:.1f} cm (ratio {left_ratio:.3f})  Right: {right_cm:.1f} cm (ratio {right_ratio:.3f})  "
          f"Discrepancy: {discrepancy:.1f} cm  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_turn_left():
    """
    Tank-turn left in continuous mode for 1 s, then verify the encoder moved.
    Arduino does NOT send DONE for LEFT/RIGHT — it runs until explicitly stopped,
    so we use a QUERY to confirm the wheels actually turned.
    """
    print("TEST: Tank-turn left (wheels off ground)...")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    send_command(ser, MOTOR, LEFT, 0.0, 0.5)  # continuous turn left
    time.sleep(1.0)
    send_command(ser, MOTOR, UP, 0.0, 0.0)    # stop
    # QUERY the encoder to confirm wheels moved
    send_command(ser, QUERY, 0, 0.0, 0.0)
    left_cm = right_cm = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            parts = line[5:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            break
    ser.close()
    if left_cm is None:
        print("  FAIL — no DIST response from QUERY")
        return False
    # During a left tank-turn, left wheel goes back and right goes forward
    passed = abs(left_cm) > 0.0 and abs(right_cm) > 0.0
    print(f"  Left: {left_cm:.2f} cm  Right: {right_cm:.2f} cm  →  {'PASS' if passed else 'FAIL (no movement detected)'}")
    return passed

def test_turn_right():
    """
    Tank-turn right in continuous mode for 1 s, then verify the encoder moved.
    Same approach as test_turn_left — QUERY used instead of waiting for DONE.
    """
    print("TEST: Tank-turn right (wheels off ground)...")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    send_command(ser, MOTOR, RIGHT, 0.0, 0.5)  # continuous turn right
    time.sleep(1.0)
    send_command(ser, MOTOR, UP, 0.0, 0.0)     # stop
    # QUERY the encoder to confirm wheels moved
    send_command(ser, QUERY, 0, 0.0, 0.0)
    left_cm = right_cm = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            parts = line[5:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            break
    ser.close()
    if left_cm is None:
        print("  FAIL — no DIST response from QUERY")
        return False
    # During a right tank-turn, right wheel goes back and left goes forward
    passed = abs(left_cm) > 0.0 and abs(right_cm) > 0.0
    print(f"  Left: {left_cm:.2f} cm  Right: {right_cm:.2f} cm  →  {'PASS' if passed else 'FAIL (no movement detected)'}")
    return passed

def test_ultrasonic_ground_detected():
    """
    Rover on flat ground — the downward ultrasonic should read < 40 cm
    (CLIFF_THRESHOLD_CM). Start a short drive and confirm DONE arrives
    without a CLIFF message.
    """
    print("TEST: Ultrasonic — ground detected during drive (place rover on flat ground)...")
    input("  Press Enter when rover is on flat ground and wheels can drive safely: ")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    send_command(ser, MOTOR, UP, 20.0, 0.5)
    result = wait_for_done(ser)
    ser.close()
    if result is None:
        print("  FAIL — no response")
        return False
    if result[0] == "CLIFF":
        print(f"  FAIL — unexpected CLIFF (left={result[1]:.1f} cm, right={result[2]:.1f} cm, sensor may not see ground)")
        return False
    left_cm, right_cm, _, _ = result
    print(f"  Drive completed — left {left_cm:.1f} cm, right {right_cm:.1f} cm, no cliff  →  PASS")
    return True

def test_ultrasonic_cliff_detected():
    """
    Simulate a cliff by lifting the front of the rover during a continuous
    drive — a CLIFF:<cm> message should arrive before DONE.
    """
    print("TEST: Ultrasonic — cliff detection (lift front of rover during drive)...")
    input("  Press Enter, then lift the front of the rover to simulate a cliff: ")
    ser = _open_and_handshake()
    if not ser:
        print("  FAIL — handshake failed")
        return False
    # Continuous drive (distance=0) so the rover keeps going until cliff is hit
    send_command(ser, MOTOR, UP, 0.0, 0.5)
    result = wait_for_done(ser, timeout=15.0)
    # Stop motors regardless of outcome
    send_command(ser, MOTOR, UP, 0.0, 0.0)
    ser.close()
    if result is None:
        print("  FAIL — no CLIFF or DONE received within 15 s")
        return False
    if result[0] == "CLIFF":
        print(f"  CLIFF detected — left {result[1]:.1f} cm, right {result[2]:.1f} cm  →  PASS")
        return True
    print(f"  FAIL — got DONE instead of CLIFF (motors stopped before cliff was triggered)")
    return False

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("NOTE: Tests 1–4 require the rover on a stand (wheels off ground).")
    print("      Tests 5–6 require the rover on the ground.\n")

    tests = [
        test_drive_forward,
        test_drive_reverse,
        test_turn_left,
        test_turn_right,
        test_ultrasonic_ground_detected,
        test_ultrasonic_cliff_detected,
    ]
    results = []
    for t in tests:
        results.append((t.__name__, t()))
        time.sleep(2.0)  # allow Arduino to reset between tests

    print("\n--- Motor + Ultrasonic Test Results ---")
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")

"""
Communication test script — Pi side.

Runs through every command type in the protocol and checks that the Arduino
responds correctly.  All tests are non-destructive: motors run at 0 speed or
for 0 distance, the servo is moved gently, and no sweep is performed.

Usage:
    python test_comms.py [--port /dev/ttyUSB0] [--baud 9600]

Exit code: 0 = all tests passed, 1 = one or more tests failed.
"""

import argparse
import sys
import time
import serial

# ── Command constants (mirror main.py) ────────────────────────────────────────
SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT, RIGHT, UP, DOWN = 0, 1, 2, 3

HANDSHAKE_TIMEOUT = 15   # seconds
RESPONSE_TIMEOUT  = 5.0  # seconds per test


# ── Helpers ───────────────────────────────────────────────────────────────────
def send(ser: serial.Serial, system: int, direction: int, amount: float, speed: float) -> None:
    packet = f"({system},{direction},{amount:.3f},{speed:.3f})\n"
    print(f"  TX: {packet.strip()}")
    ser.write(packet.encode())


def read_line(ser: serial.Serial, timeout: float = RESPONSE_TIMEOUT) -> str:
    """Read one line from serial with a timeout; return '' on timeout."""
    ser.timeout = timeout
    line = ser.readline().decode(errors="replace").strip()
    if line:
        print(f"  RX: {line}")
    return line


def drain(ser: serial.Serial, seconds: float = 0.3) -> None:
    """Discard any pending bytes."""
    time.sleep(seconds)
    ser.reset_input_buffer()


# ── Handshake ─────────────────────────────────────────────────────────────────
def handshake(ser: serial.Serial) -> bool:
    print("[TEST] Handshake (READY / ACK)")
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = read_line(ser, timeout=2.0)
        if line == "READY":
            ser.write(b"ACK\n")
            print("  TX: ACK")
            print("  PASS — handshake complete\n")
            return True
    print("  FAIL — timed out waiting for READY\n")
    return False


# ── Individual tests ──────────────────────────────────────────────────────────
def test_servo(ser: serial.Serial) -> bool:
    """Send a servo command; Arduino echoes 'SERVO_OK:<angle>' in test firmware."""
    print("[TEST] Servo command  (system=0, dir=UP, amount=0.5 → 90°)")
    drain(ser)
    send(ser, SERVO, UP, 0.5, 0.0)
    line = read_line(ser)
    if line.startswith("SERVO_OK:"):
        print("  PASS\n")
        return True
    print(f"  FAIL — unexpected response: '{line}'\n")
    return False


def test_motor_continuous(ser: serial.Serial) -> bool:
    """Send continuous motor command (distance=0); Arduino echoes 'MOTOR_OK'."""
    print("[TEST] Motor continuous (system=1, dir=UP, distance=0, speed=0)")
    drain(ser)
    send(ser, MOTOR, UP, 0.0, 0.0)
    line = read_line(ser)
    if line == "MOTOR_OK":
        print("  PASS\n")
        return True
    print(f"  FAIL — unexpected response: '{line}'\n")
    return False


def test_motor_distance(ser: serial.Serial) -> bool:
    """Send distance-controlled command; Arduino should respond with DONE:."""
    print("[TEST] Motor distance  (system=1, dir=UP, distance=1cm, speed=0)")
    drain(ser)
    send(ser, MOTOR, UP, 1.0, 0.0)
    deadline = time.monotonic() + RESPONSE_TIMEOUT
    while time.monotonic() < deadline:
        line = read_line(ser)
        if line.startswith("DONE:"):
            parts = line[5:].split(",")
            if len(parts) == 2:
                try:
                    float(parts[0])
                    float(parts[1])
                    print("  PASS\n")
                    return True
                except ValueError:
                    pass
    print(f"  FAIL — did not receive valid DONE: response\n")
    return False


def test_query(ser: serial.Serial) -> bool:
    """Send a QUERY command; Arduino should respond with DIST:."""
    print("[TEST] Query  (system=3)")
    drain(ser)
    send(ser, QUERY, 0, 0.0, 0.0)
    deadline = time.monotonic() + RESPONSE_TIMEOUT
    while time.monotonic() < deadline:
        line = read_line(ser)
        if line.startswith("DIST:"):
            try:
                float(line[5:])
                print("  PASS\n")
                return True
            except ValueError:
                pass
    print("  FAIL — did not receive valid DIST: response\n")
    return False


def test_flip_on(ser: serial.Serial) -> bool:
    """Send flip-enable command; Arduino echoes 'FLIP_OK:1' in test firmware."""
    print("[TEST] Flip ON  (system=4, amount=1.0)")
    drain(ser)
    send(ser, FLIP, 0, 1.0, 0.0)
    line = read_line(ser)
    if line == "FLIP_OK:1":
        print("  PASS\n")
        return True
    print(f"  FAIL — unexpected response: '{line}'\n")
    return False


def test_flip_off(ser: serial.Serial) -> bool:
    """Send flip-disable command; Arduino echoes 'FLIP_OK:0'."""
    print("[TEST] Flip OFF  (system=4, amount=0.0)")
    drain(ser)
    send(ser, FLIP, 0, 0.0, 0.0)
    line = read_line(ser)
    if line == "FLIP_OK:0":
        print("  PASS\n")
        return True
    print(f"  FAIL — unexpected response: '{line}'\n")
    return False


def test_bad_packet(ser: serial.Serial) -> bool:
    """Send a malformed packet; Arduino should echo 'PARSE_ERR'."""
    print("[TEST] Bad packet  (malformed string)")
    drain(ser)
    ser.write(b"garbage\n")
    print("  TX: garbage")
    line = read_line(ser)
    if line == "PARSE_ERR":
        print("  PASS\n")
        return True
    print(f"  FAIL — unexpected response: '{line}'\n")
    return False


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(description="Pi ↔ Arduino communication test")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=9600,  help="Baud rate")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud…")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=2.0)
    except serial.SerialException as exc:
        print(f"ERROR: {exc}")
        return 1

    time.sleep(2)  # let Arduino reset after DTR toggle
    ser.reset_input_buffer()

    results: list[tuple[str, bool]] = []

    def run(name: str, fn) -> None:
        passed = fn(ser)
        results.append((name, passed))

    if not handshake(ser):
        print("Aborting — handshake failed.")
        ser.close()
        return 1

    run("Servo command",         test_servo)
    run("Motor continuous",      test_motor_continuous)
    run("Motor distance/DONE",   test_motor_distance)
    run("Query/DIST",            test_query)
    run("Flip ON",               test_flip_on)
    run("Flip OFF",              test_flip_off)
    run("Bad-packet PARSE_ERR",  test_bad_packet)

    ser.close()

    print("=" * 50)
    print("RESULTS")
    print("=" * 50)
    passed = sum(1 for _, ok in results if ok)
    for name, ok in results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}]  {name}")
    print(f"\n{passed}/{len(results)} tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

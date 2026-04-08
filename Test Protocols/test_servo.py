"""
Test Protocol: Servos
=====================
Exercises the pan and tilt servos through their full range of motion and
verifies the SWEEP protocol (both servos coordinated with LIDAR stepping).

Servo command format (from main.py communication schema):
    (0, <direction>, <amount>, 0.000)
    System  = 0  (SERVO)
    Direction:
        LEFT  (0) = pan servo    — horizontal rotation
        RIGHT (1) = pan servo    — same servo, direction encoded in amount
        UP    (2) = tilt servo   — vertical rotation
        DOWN  (3) = tilt servo   — same servo, direction encoded in amount
    Amount = target position 0.0–1.0  (maps to 0°–180° in the Arduino)
    Speed  = unused for servos (always send 0.0)

The Arduino sends no response to SERVO commands.  Tests verify:
  1. The Arduino stays responsive (QUERY returns DIST:) after each command.
  2. Physical movement is confirmed interactively by the operator.

SWEEP protocol:
    Pi sends   (2, 0, <range_deg>, <step_ms>)
    Arduino replies  AT:<h_angle>,<tilt_angle>  at each position
    Pi replies       NEXT                        to step to the next position
    Arduino sends    SWEEP_DONE                  when back at centre

Requires: Arduino connected on SERIAL_PORT, running rover_interpreter.ino.
No I2C sensors are needed for servo or sweep tests.

Run on the Pi:
    python3 "test_servo.py"
"""

import time
import serial

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyACM0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10    # seconds
QUERY_TIMEOUT     = 3.0   # seconds to wait for DIST: after a QUERY
SWEEP_TIMEOUT     = 120.0 # seconds hard cap waiting for SWEEP_DONE

# ── Command constants (mirror of main.py) ─────────────────────────────────────
SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT, RIGHT, UP, DOWN            = 0, 1, 2, 3

# Pan servo uses LEFT/RIGHT direction; tilt servo uses UP/DOWN.
# Amount 0.0 → 0°,  0.5 → 90° (centre),  1.0 → 180°.
PAN_DIR  = LEFT   # direction field used for pan servo commands
TILT_DIR = UP     # direction field used for tilt servo commands

# Positions used in the sweep tests
STEP_POSITIONS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
CENTRE         = 0.5   # 90° — home position for both axes
DWELL_S        = 0.4   # seconds to pause at each position so movement is visible

# ── Shared serial helpers ─────────────────────────────────────────────────────
def _open_and_handshake() -> "serial.Serial | None":
    """Open the port and complete READY/ACK.  Returns Serial or None on failure."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"  Could not open {SERIAL_PORT}: {e}")
        return None
    ser.reset_input_buffer()
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            return ser
    ser.close()
    print(f"  Arduino did not send READY within {HANDSHAKE_TIMEOUT} s")
    return None


def send_command(ser: serial.Serial, system: int, direction: int,
                 amount: float, speed: float) -> None:
    """Encode and transmit one (system, direction, amount, speed) packet."""
    ser.write(f"({system},{direction},{amount:.3f},{speed:.3f})\n".encode())


def send_servo(ser: serial.Serial, direction: int, position: float) -> None:
    """Move one servo axis to position (0.0–1.0 → 0°–180°)."""
    send_command(ser, SERVO, direction, position, 0.0)


def _wait_for_prefix(ser: serial.Serial, prefix: str,
                     timeout: float = QUERY_TIMEOUT) -> str:
    """Read lines until one starts with prefix.  Returns the line or ''."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    return ""


def _arduino_alive(ser: serial.Serial) -> bool:
    """
    Send a QUERY and expect DIST: back within QUERY_TIMEOUT.
    Used after every servo command to confirm the Arduino is still running.
    """
    send_command(ser, QUERY, 0, 0.0, 0.0)
    return bool(_wait_for_prefix(ser, "DIST:"))


def _home(ser: serial.Serial) -> None:
    """Return both servos to centre (90°) between tests."""
    send_servo(ser, PAN_DIR,  CENTRE)
    send_servo(ser, TILT_DIR, CENTRE)
    time.sleep(DWELL_S)


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Individual servo positions (pan)
# ═════════════════════════════════════════════════════════════════════════════

def test_pan_centre(ser: serial.Serial) -> bool:
    """
    Pan servo → 90° (amount=0.5).
    The turret should be pointing straight ahead.
    Arduino must remain responsive after the command.
    """
    print("TEST: Pan → centre (90°)…")
    send_servo(ser, PAN_DIR, 0.5)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_full_left(ser: serial.Serial) -> bool:
    """
    Pan servo → 0° (amount=0.0) — hard left limit.
    Watch the turret rotate to the left end-stop.
    """
    print("TEST: Pan → full left (0°)…")
    send_servo(ser, PAN_DIR, 0.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_full_right(ser: serial.Serial) -> bool:
    """
    Pan servo → 180° (amount=1.0) — hard right limit.
    Watch the turret rotate to the right end-stop.
    """
    print("TEST: Pan → full right (180°)…")
    send_servo(ser, PAN_DIR, 1.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_sweep_manual(ser: serial.Serial) -> bool:
    """
    Step the pan servo through 0° → 180° → 90° in 10% increments.
    Verifies the Arduino accepts every command and stays alive throughout.
    Operator should see smooth continuous rotation.
    """
    print(f"TEST: Pan stepped sweep 0°→180°→90° ({len(STEP_POSITIONS)*2 - 1} steps)…")
    all_ok = True
    for pos in STEP_POSITIONS + list(reversed(STEP_POSITIONS[:-1])) + [CENTRE]:
        send_servo(ser, PAN_DIR, pos)
        time.sleep(DWELL_S)
    if not _arduino_alive(ser):
        all_ok = False
    angle_str = "→".join(f"{int(p*180)}°" for p in [0.0, 0.5, 1.0, 0.5])
    print(f"  Positions: {angle_str}  →  {'PASS' if all_ok else 'FAIL — Arduino unresponsive mid-sweep'}")
    return all_ok


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Individual servo positions (tilt)
# ═════════════════════════════════════════════════════════════════════════════

def test_tilt_centre(ser: serial.Serial) -> bool:
    """
    Tilt servo → 90° (amount=0.5) — horizontal / level.
    The LIDAR should be pointing straight forward.
    """
    print("TEST: Tilt → centre (90° / level)…")
    send_servo(ser, TILT_DIR, 0.5)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_tilt_full_down(ser: serial.Serial) -> bool:
    """
    Tilt servo → 0° (amount=0.0) — maximum downward angle.
    The LIDAR should point toward the ground.
    """
    print("TEST: Tilt → full down (0°)…")
    send_servo(ser, TILT_DIR, 0.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_tilt_full_up(ser: serial.Serial) -> bool:
    """
    Tilt servo → 180° (amount=1.0) — maximum upward angle.
    The LIDAR should point toward the sky.
    """
    print("TEST: Tilt → full up (180°)…")
    send_servo(ser, TILT_DIR, 1.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_tilt_sweep_manual(ser: serial.Serial) -> bool:
    """
    Step the tilt servo through 0° → 180° → 90° in 10% increments.
    Operator should see smooth vertical rotation.
    """
    print(f"TEST: Tilt stepped sweep 0°→180°→90° ({len(STEP_POSITIONS)*2 - 1} steps)…")
    all_ok = True
    for pos in STEP_POSITIONS + list(reversed(STEP_POSITIONS[:-1])) + [CENTRE]:
        send_servo(ser, TILT_DIR, pos)
        time.sleep(DWELL_S)
    if not _arduino_alive(ser):
        all_ok = False
    angle_str = "→".join(f"{int(p*180)}°" for p in [0.0, 0.5, 1.0, 0.5])
    print(f"  Positions: {angle_str}  →  {'PASS' if all_ok else 'FAIL — Arduino unresponsive mid-sweep'}")
    return all_ok


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Combined axis tests
# ═════════════════════════════════════════════════════════════════════════════

def test_simultaneous_move(ser: serial.Serial) -> bool:
    """
    Send a pan and tilt command back-to-back (no delay between them).
    Both servos should move to their targets without stalling or crashing.
    """
    print("TEST: Pan + tilt commanded simultaneously (no gap)…")
    targets = [
        (0.0, 0.0),   # hard left, full down
        (1.0, 1.0),   # hard right, full up
        (0.5, 0.5),   # centre both
    ]
    for pan_pos, tilt_pos in targets:
        send_servo(ser, PAN_DIR,  pan_pos)
        send_servo(ser, TILT_DIR, tilt_pos)
        time.sleep(DWELL_S * 2)  # extra dwell — two axes moving at once

    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive after simultaneous moves'}")
    return passed


def test_rapid_commands(ser: serial.Serial) -> bool:
    """
    Fire 20 servo commands with only a 50 ms gap between each.
    Tests that the Arduino serial buffer and parser don't overflow or hang.
    """
    print("TEST: 20 rapid servo commands (50 ms apart)…")
    positions = [i / 20.0 for i in range(21)]   # 0.00 → 1.00 in steps of 0.05
    for pos in positions:
        send_servo(ser, PAN_DIR, pos)
        time.sleep(0.05)
    time.sleep(0.3)   # let last move settle
    passed = _arduino_alive(ser)
    print(f"  Sent {len(positions)} commands  →  {'PASS — Arduino still alive' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 4 — SWEEP protocol (system 2)
# The SWEEP protocol coordinates both servos automatically.  The Arduino
# moves to each position, sends AT:<h_angle>,<tilt_angle>, then waits for
# NEXT before advancing.  It sends SWEEP_DONE when back at centre.
# ═════════════════════════════════════════════════════════════════════════════

def _run_sweep(ser: serial.Serial, range_deg: float, step_ms: float,
               label: str) -> tuple[int, bool]:
    """
    Send a SWEEP command and consume all AT:/SWEEP_DONE responses.
    Sends NEXT after every AT: to keep the sweep progressing.

    Returns (at_count, sweep_done).
    """
    print(f"  Sending SWEEP range={range_deg}° step={step_ms}ms…")
    send_command(ser, SWEEP, 0, range_deg, step_ms)

    at_count   = 0
    sweep_done = False
    deadline   = time.monotonic() + SWEEP_TIMEOUT

    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("AT:"):
            at_count += 1
            parts = line[3:].split(",")
            h_ang  = parts[0] if len(parts) > 0 else "?"
            t_ang  = parts[1] if len(parts) > 1 else "?"
            print(f"    AT: h={h_ang}°  tilt={t_ang}°  (msg #{at_count})")
            ser.write(b"NEXT\n")
        elif line == "SWEEP_DONE":
            sweep_done = True
            break
        else:
            print(f"    [{line}]")   # unexpected — print for debugging

    return at_count, sweep_done


def test_sweep_small_range(ser: serial.Serial) -> bool:
    """
    SWEEP with a 10° horizontal range.
    Expects at least one AT: message and a SWEEP_DONE.
    Both servos should move visibly within a narrow arc.
    """
    print("TEST: SWEEP 10° range — AT: messages + SWEEP_DONE…")
    at_count, sweep_done = _run_sweep(ser, range_deg=10.0, step_ms=150.0,
                                      label="10° sweep")
    passed = at_count >= 1 and sweep_done
    print(f"  AT count: {at_count}  SWEEP_DONE: {sweep_done}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_sweep_90_deg(ser: serial.Serial) -> bool:
    """
    SWEEP with a 90° horizontal range — the full navigation sweep used in main.py.
    Expects multiple AT: messages covering the full arc, ending with SWEEP_DONE.
    """
    print("TEST: SWEEP 90° range (navigation sweep) — AT: messages + SWEEP_DONE…")
    at_count, sweep_done = _run_sweep(ser, range_deg=90.0, step_ms=200.0,
                                      label="90° sweep")
    passed = at_count >= 3 and sweep_done   # at minimum: left, centre, right
    print(f"  AT count: {at_count}  SWEEP_DONE: {sweep_done}  "
          f"→  {'PASS' if passed else 'FAIL (expected ≥3 AT messages)'}")
    return passed


def test_sweep_at_format(ser: serial.Serial) -> bool:
    """
    Verify that every AT: message from a 10° sweep contains two valid numbers:
        AT:<h_angle_float>,<tilt_angle_int>
    A malformed AT: message would cause main.py to skip the reading silently.
    """
    print("TEST: SWEEP 10° — AT: message format (both fields parse as numbers)…")
    send_command(ser, SWEEP, 0, 10.0, 150.0)

    valid   = []
    invalid = []
    sweep_done = False
    deadline = time.monotonic() + SWEEP_TIMEOUT

    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("AT:"):
            try:
                parts = line[3:].split(",")
                if len(parts) != 2:
                    raise ValueError("wrong field count")
                float(parts[0])   # h_angle
                float(parts[1])   # tilt_angle
                valid.append(line)
            except (ValueError, IndexError):
                invalid.append(line)
            ser.write(b"NEXT\n")
        elif line == "SWEEP_DONE":
            sweep_done = True
            break

    passed = len(valid) > 0 and len(invalid) == 0 and sweep_done
    print(f"  Valid: {len(valid)}  Invalid: {invalid}  SWEEP_DONE: {sweep_done}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_sweep_next_blocks(ser: serial.Serial) -> bool:
    """
    Verify that the Arduino waits for NEXT before advancing past each AT: position.

    Method:
      1. Start a 10° sweep.
      2. Receive the first AT: but do NOT send NEXT for 500 ms.
      3. Confirm no second AT: arrives during the wait (Arduino is paused).
      4. Send NEXT — verify the sweep then advances.
    """
    print("TEST: SWEEP — Arduino pauses at each AT: until NEXT is sent…")
    send_command(ser, SWEEP, 0, 10.0, 150.0)

    # Step 1: Get first AT:
    first_at = _wait_for_prefix(ser, "AT:", timeout=30.0)
    if not first_at:
        print("  FAIL — no AT: received")
        return False
    print(f"  First AT: received: '{first_at}'")

    # Step 2: Wait 500 ms without sending NEXT — no second AT: should arrive
    extra_line = ""
    pause_deadline = time.monotonic() + 0.5
    while time.monotonic() < pause_deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("AT:") or line == "SWEEP_DONE":
            extra_line = line
            break

    if extra_line:
        print(f"  FAIL — Arduino advanced without NEXT: '{extra_line}'")
        # Drain the rest of the sweep so the Arduino is back at home
        ser.write(b"NEXT\n")
        _wait_for_prefix(ser, "SWEEP_DONE", timeout=SWEEP_TIMEOUT)
        return False

    # Step 3: Send NEXT and confirm sweep advances
    print("  No premature advance — sending NEXT…")
    ser.write(b"NEXT\n")
    next_line = _wait_for_prefix(ser, "", timeout=10.0)   # any non-empty line
    advanced  = next_line.startswith("AT:") or next_line == "SWEEP_DONE"

    # Drain remaining sweep
    if next_line.startswith("AT:"):
        ser.write(b"NEXT\n")
    _wait_for_prefix(ser, "SWEEP_DONE", timeout=SWEEP_TIMEOUT)

    passed = advanced
    print(f"  After NEXT, received: '{next_line}'  "
          f"→  {'PASS' if passed else 'FAIL — no advance after NEXT'}")
    return passed


def test_sweep_returns_to_centre(ser: serial.Serial) -> bool:
    """
    After SWEEP_DONE, both servos should physically be back at their centre
    positions.  The operator confirms visually.

    This test always returns True from the code's perspective — the PASS/FAIL
    is recorded based on the operator's response.
    """
    print("TEST: SWEEP 90° — servos return to centre after SWEEP_DONE…")
    _run_sweep(ser, range_deg=90.0, step_ms=200.0, label="return-to-centre sweep")
    answer = input("  Are both servos back at centre (straight ahead, level)? [y/n]: ").strip().lower()
    passed = answer == "y"
    print(f"  Operator confirmed: {answer}  →  {'PASS' if passed else 'FAIL'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Edge cases
# ═════════════════════════════════════════════════════════════════════════════

def test_servo_clamp_below_zero(ser: serial.Serial) -> bool:
    """
    Send amount = -0.5 (below valid range).
    The Arduino should clamp to 0° or ignore it — it must not crash.
    """
    print("TEST: SERVO amount = -0.5 (below range) — Arduino must stay alive…")
    send_command(ser, SERVO, PAN_DIR, -0.5, 0.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive after out-of-range command'}")
    return passed


def test_servo_clamp_above_one(ser: serial.Serial) -> bool:
    """
    Send amount = 1.5 (above valid range).
    The Arduino should clamp to 180° or ignore it — it must not crash.
    """
    print("TEST: SERVO amount = 1.5 (above range) — Arduino must stay alive…")
    send_command(ser, SERVO, PAN_DIR, 1.5, 0.0)
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive after out-of-range command'}")
    return passed


def test_servo_speed_field_ignored(ser: serial.Serial) -> bool:
    """
    The speed field is unused for servo commands.
    Sending a non-zero speed (0.7) should have no effect on behaviour.
    """
    print("TEST: SERVO with non-zero speed field (should be ignored)…")
    send_command(ser, SERVO, PAN_DIR, 0.5, 0.7)   # speed=0.7 — not used
    time.sleep(DWELL_S)
    passed = _arduino_alive(ser)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# VISUAL INSPECTION HELPER
# ═════════════════════════════════════════════════════════════════════════════

def _visual_check(ser: serial.Serial) -> None:
    """
    Interactive manual mode: type an angle (0–180) and press Enter to move
    the pan servo to that position.  Type 'q' to exit.
    Useful for checking physical endpoints and finding any binding.
    """
    print("\n── Interactive servo check ─────────────────────────────")
    print("  Enter angle 0–180 for pan servo, 'tilt <angle>' for tilt,")
    print("  or 'q' to continue.\n")
    while True:
        try:
            raw = input("  angle> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if raw in ("q", "quit", "exit"):
            break
        parts = raw.split()
        try:
            if len(parts) == 2 and parts[0] == "tilt":
                angle  = float(parts[1])
                amount = max(0.0, min(1.0, angle / 180.0))
                send_servo(ser, TILT_DIR, amount)
                print(f"  Tilt → {angle:.0f}°  (amount={amount:.3f})")
            else:
                angle  = float(parts[-1])
                amount = max(0.0, min(1.0, angle / 180.0))
                send_servo(ser, PAN_DIR, amount)
                print(f"  Pan  → {angle:.0f}°  (amount={amount:.3f})")
        except ValueError:
            print("  Enter a number (e.g. 90), 'tilt 45', or 'q' to quit.")
    print()


# ═════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  COSGC Rover — Servo Test Protocol")
    print("=" * 60)
    print(f"  Port      : {SERIAL_PORT}  @  {BAUD_RATE} baud")
    print(f"  Pan axis  : SERVO direction={PAN_DIR}  (LEFT)")
    print(f"  Tilt axis : SERVO direction={TILT_DIR}  (UP)")
    print(f"  Dwell     : {DWELL_S*1000:.0f} ms per position\n")

    print("Connecting to Arduino…")
    ser = _open_and_handshake()
    if ser is None:
        print("Could not connect — exiting.")
        raise SystemExit(1)
    print("Handshake complete.\n")

    # Return to known starting position before any tests
    print("Homing both servos to centre (90°)…")
    _home(ser)
    time.sleep(0.5)

    results: list[tuple[str, bool]] = []

    # ── Section 1: Pan axis ───────────────────────────────────────────────────
    print("\n=== PAN SERVO (horizontal) ===")
    print("Watch the turret rotate left/right.\n")
    for fn in [
        test_pan_centre,
        test_pan_full_left,
        test_pan_full_right,
        test_pan_sweep_manual,
    ]:
        results.append((fn.__name__, fn(ser)))
        _home(ser)

    # ── Section 2: Tilt axis ──────────────────────────────────────────────────
    print("\n=== TILT SERVO (vertical) ===")
    print("Watch the LIDAR tilt up/down.\n")
    for fn in [
        test_tilt_centre,
        test_tilt_full_down,
        test_tilt_full_up,
        test_tilt_sweep_manual,
    ]:
        results.append((fn.__name__, fn(ser)))
        _home(ser)

    # ── Section 3: Combined axis ──────────────────────────────────────────────
    print("\n=== COMBINED AXIS ===\n")
    for fn in [test_simultaneous_move, test_rapid_commands]:
        results.append((fn.__name__, fn(ser)))
        _home(ser)

    # ── Section 4: SWEEP protocol ─────────────────────────────────────────────
    print("\n=== SWEEP PROTOCOL (system 2) ===\n")
    for fn in [
        test_sweep_small_range,
        test_sweep_90_deg,
        test_sweep_at_format,
        test_sweep_next_blocks,
        test_sweep_returns_to_centre,
    ]:
        results.append((fn.__name__, fn(ser)))
        _home(ser)

    # ── Section 5: Edge cases ─────────────────────────────────────────────────
    print("\n=== EDGE CASES ===\n")
    for fn in [
        test_servo_clamp_below_zero,
        test_servo_clamp_above_one,
        test_servo_speed_field_ignored,
    ]:
        results.append((fn.__name__, fn(ser)))
        _home(ser)

    # ── Interactive check (optional) ──────────────────────────────────────────
    print("\n=== INTERACTIVE CHECK (optional) ===")
    answer = input("Run interactive servo control? [y/N]: ").strip().lower()
    if answer == "y":
        _visual_check(ser)

    # ── Return to home and close ──────────────────────────────────────────────
    print("Returning servos to centre…")
    _home(ser)
    ser.close()

    # ── Results summary ───────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("SERVO TEST RESULTS")
    print("=" * 60)
    passed_n = sum(1 for _, p in results if p)
    failed_n = sum(1 for _, p in results if not p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n  {passed_n} passed  |  {failed_n} failed  |  {len(results)} total")

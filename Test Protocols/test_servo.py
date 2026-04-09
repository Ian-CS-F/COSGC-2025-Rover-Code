"""
Test Protocol: Servos
=====================
Exercises the tilt servo and motor-based pan through their full range of
motion, and verifies the SWEEP protocol.

Hardware configuration:
    Tilt axis — single physical servo on PIN_TILT (pin 12).
                Controlled via SERVO command with direction UP or DOWN.
    Pan axis  — no pan servo fitted.  Panning rotates the WHOLE ROVER using
                a motor tank-turn.  The Arduino tracks the current pan angle
                and only drives the delta on each command.

Servo command format:
    (0, <direction>, <amount>, <speed>)
    System  = 0  (SERVO)
    Direction:
        LEFT  (0) / RIGHT (1) = pan  — tank-turns the rover
        UP    (2) / DOWN  (3) = tilt — moves the physical tilt servo
    Amount = target angle 0.0–1.0  (→ 0°–180°)
               0.0 = full left / full down
               0.5 = centre (90°)
               1.0 = full right / full up
    Speed  = motor PWM for pan turns (0.0–1.0); ignored for tilt

The Arduino sends no response to SERVO commands.  Tests verify:
  1. The Arduino stays responsive (QUERY returns DIST:) after each command.
  2. Physical movement is confirmed interactively by the operator.

PAN TESTS ROTATE THE ROVER — ensure the rover has clear space on all sides.

SWEEP protocol:
    Pi sends   (2, 0, <range_deg>, <step_ms>)
    Arduino replies  AT:<h_angle>,<tilt_angle>  at each position
    Pi replies       NEXT                        to step to the next position
    Arduino sends    SWEEP_DONE                  when back at centre
    (The sweep uses the same motor tank-turn for horizontal steps.)

Requires: Arduino connected on SERIAL_PORT, running rover_interpreter.ino.
No I2C sensors are needed for servo or sweep tests.

Run on the Pi:
    python3 "test_servo.py"
"""

import math
import struct
import time
import serial

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyACM0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10    # seconds
QUERY_TIMEOUT     = 3.0   # seconds to wait for DIST: after a QUERY
PAN_DONE_TIMEOUT  = 15.0  # seconds to wait for PAN_DONE (covers longest turn)
SWEEP_TIMEOUT     = 120.0 # seconds hard cap waiting for SWEEP_DONE

# Set True to read the IMU before/after each pan and report actual vs expected.
# Requires ICM-20948 connected to the Pi's I2C bus 1.  Set False to skip.
VERIFY_WITH_IMU = True

# Acceptable heading error for the IMU comparison (degrees).
# The time-based turn (MS_PER_DEGREE) won't be perfect — this is just a flag.
IMU_TOLERANCE_DEG = 15.0

# ── Command constants (mirror of main.py) ─────────────────────────────────────
SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT, RIGHT, UP, DOWN            = 0, 1, 2, 3

# Pan uses LEFT/RIGHT direction (tank-turn via motors).
# Tilt uses UP/DOWN direction (physical servo).
# Amount 0.0 → 0°,  0.5 → 90° (centre),  1.0 → 180°.
PAN_DIR  = LEFT
TILT_DIR = UP

# Positions used in the sweep tests
STEP_POSITIONS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
CENTRE         = 0.5   # 90° — home position for both axes
DWELL_S        = 0.4   # seconds to pause at each position (tilt tests)

# ── IMU constants (ICM-20948 + AK09916) ──────────────────────────────────────
I2C_BUS      = 1
ICM_ADDR     = 0x69
MAG_ADDR     = 0x0C
REG_BANK_SEL = 0x7F
PWR_MGMT_1   = 0x06
INT_PIN_CFG  = 0x0F
ACCEL_XOUT_H = 0x2D
MAG_CNTL2    = 0x31
MAG_ST1      = 0x10
MAG_HXL      = 0x11

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


def _wait_for_pan_done(ser: serial.Serial) -> float | None:
    """
    Block until the Arduino sends PAN_DONE:<delta_deg> and return the
    estimated delta (positive = turned right, negative = turned left).
    Returns None on timeout.
    """
    line = _wait_for_prefix(ser, "PAN_DONE:", timeout=PAN_DONE_TIMEOUT)
    if not line:
        return None
    try:
        return float(line[9:])
    except ValueError:
        return None


# ── IMU helpers ───────────────────────────────────────────────────────────────

def _init_imu(bus) -> bool:
    """Wake ICM-20948 and enable I2C bypass for the AK09916 magnetometer."""
    try:
        bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, 0x00)
        bus.write_byte_data(ICM_ADDR, PWR_MGMT_1,   0x01)   # wake, auto clock
        time.sleep(0.1)
        bus.write_byte_data(ICM_ADDR, INT_PIN_CFG,  0x02)   # I2C bypass
        time.sleep(0.02)
        bus.write_byte_data(MAG_ADDR, MAG_CNTL2,    0x08)   # AK09916 100 Hz
        time.sleep(0.02)
        return True
    except OSError:
        return False


def _read_heading(bus) -> float | None:
    """
    Return tilt-compensated compass heading in degrees (0–360, 0 = North).
    Returns None if the IMU or magnetometer is unresponsive.
    """
    try:
        bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, 0x00)
        raw = bus.read_i2c_block_data(ICM_ADDR, ACCEL_XOUT_H, 6)
        ax = struct.unpack(">h", bytes(raw[0:2]))[0] / 16384.0
        ay = struct.unpack(">h", bytes(raw[2:4]))[0] / 16384.0
        az = struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0

        # Wait for magnetometer data-ready
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            if bus.read_byte_data(MAG_ADDR, MAG_ST1) & 0x01:
                break
        raw = bus.read_i2c_block_data(MAG_ADDR, MAG_HXL, 8)
        mx = struct.unpack("<h", bytes(raw[0:2]))[0]
        my = struct.unpack("<h", bytes(raw[2:4]))[0]
        mz = struct.unpack("<h", bytes(raw[4:6]))[0]

        roll  = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        mx_c  = mx * math.cos(pitch) + mz * math.sin(pitch)
        my_c  = (mx * math.sin(roll) * math.sin(pitch)
                 + my * math.cos(roll)
                 - mz * math.sin(roll) * math.cos(pitch))
        return math.degrees(math.atan2(-my_c, mx_c)) % 360
    except OSError:
        return None


def _heading_delta(before: float, after: float) -> float:
    """
    Signed heading change in degrees, normalised to −180 … +180.
    Positive = turned right (clockwise), negative = turned left.
    """
    return ((after - before) + 180.0) % 360.0 - 180.0


def _imu_summary(expected_deg: float | None, actual_deg: float | None) -> str:
    """Build a one-line IMU comparison string for test output."""
    if actual_deg is None:
        return "IMU: no reading"
    exp_str = f"{expected_deg:+.1f}°" if expected_deg is not None else "?"
    err = abs(actual_deg - expected_deg) if expected_deg is not None else None
    ok  = err is not None and err <= IMU_TOLERANCE_DEG
    flag = "✓" if ok else f"! (err={err:.1f}°)"
    return f"IMU: expected {exp_str}  actual {actual_deg:+.1f}°  {flag}"


# ── Pan helper: send command and verify with IMU ──────────────────────────────

def _pan(ser: serial.Serial, position: float,
         imu_bus=None) -> tuple[bool, float | None]:
    """
    Send a SERVO pan command, wait for PAN_DONE, then read the IMU.

    Returns (arduino_alive, actual_heading_delta).
    actual_heading_delta is None when IMU is not available.
    """
    heading_before = _read_heading(imu_bus) if imu_bus else None

    send_servo(ser, PAN_DIR, position)
    estimated_delta = _wait_for_pan_done(ser)   # blocks until Arduino finishes

    heading_after = _read_heading(imu_bus) if imu_bus else None

    actual_delta: float | None = None
    if heading_before is not None and heading_after is not None:
        actual_delta = _heading_delta(heading_before, heading_after)

    if estimated_delta is not None and actual_delta is not None:
        print(f"  {_imu_summary(estimated_delta, actual_delta)}")
    elif estimated_delta is not None:
        print(f"  Arduino estimate: {estimated_delta:+.1f}°  (IMU not available)")

    alive = _arduino_alive(ser)
    return alive, actual_delta


def _home(ser: serial.Serial, imu_bus=None) -> None:
    """Return both axes to centre (90°) between tests."""
    _pan(ser, CENTRE, imu_bus)
    send_servo(ser, TILT_DIR, CENTRE)
    time.sleep(DWELL_S)


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Individual servo positions (pan)
# ═════════════════════════════════════════════════════════════════════════════

def test_pan_centre(ser: serial.Serial, imu_bus=None) -> bool:
    """
    Pan → 90° (amount=0.5) — rover faces straight ahead.
    Waits for PAN_DONE, then reads IMU to compare actual vs expected delta.
    """
    print("TEST: Pan → centre (90°) — rover turns to face forward…")
    passed, _ = _pan(ser, 0.5, imu_bus)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_full_left(ser: serial.Serial, imu_bus=None) -> bool:
    """
    Pan → 0° (amount=0.0) — rover rotates 90° left of centre.
    IMU reports actual heading change vs the −90° expected delta.
    """
    print("TEST: Pan → full left (0°) — rover rotates left…")
    passed, _ = _pan(ser, 0.0, imu_bus)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_full_right(ser: serial.Serial, imu_bus=None) -> bool:
    """
    Pan → 180° (amount=1.0) — rover rotates right (180° from left end).
    IMU reports actual heading change vs the +180° expected delta.
    """
    print("TEST: Pan → full right (180°) — rover rotates right…")
    passed, _ = _pan(ser, 1.0, imu_bus)
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive'}")
    return passed


def test_pan_sweep_manual(ser: serial.Serial, imu_bus=None) -> bool:
    """
    Step the pan through 0° → 180° → 90° in 10% increments via motor turns.
    The rover physically rotates at each step — ensure 180° of clear space.
    IMU heading is read before and after each step and compared to the
    Arduino's time-based estimate.  Overall result checks Arduino stays alive.
    """
    print(f"TEST: Pan stepped sweep 0°→180°→90° ({len(STEP_POSITIONS)*2 - 1} steps)…")
    print("  NOTE: rover will physically rotate — ensure clear space around it.")
    all_ok = True
    positions = STEP_POSITIONS + list(reversed(STEP_POSITIONS[:-1])) + [CENTRE]
    for pos in positions:
        alive, _ = _pan(ser, pos, imu_bus)
        if not alive:
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
    Verifies the Arduino serial parser doesn't choke when two commands arrive
    in quick succession.

    NOTE: Execution is sequential, not truly simultaneous.  Pan (motor tank-
    turn) blocks the Arduino for ~200–1000 ms while it runs the motors; tilt
    is processed after the pan completes.  The test checks that the Arduino
    remains responsive after both commands, not that the axes move in parallel.
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
    Fire 20 TILT servo commands with only a 50 ms gap between each.
    Tests that the Arduino serial buffer and parser don't overflow or hang.

    NOTE: Uses TILT_DIR (physical servo, responds instantly) — NOT PAN_DIR.
    Pan uses motor tank-turns that take ~200 ms per step; sending pan commands
    at 50 ms intervals would overflow the serial buffer before the Arduino can
    process them.  This test is about serial parsing throughput, not motor
    behaviour, so tilt is the correct axis to stress here.
    """
    print("TEST: 20 rapid TILT commands (50 ms apart)…")
    positions = [i / 20.0 for i in range(21)]   # 0.00 → 1.00 in steps of 0.05
    for pos in positions:
        send_servo(ser, TILT_DIR, pos)
        time.sleep(0.05)
    time.sleep(0.3)   # let last move settle
    passed = _arduino_alive(ser)
    print(f"  Sent {len(positions)} tilt commands  →  {'PASS — Arduino still alive' if passed else 'FAIL — Arduino unresponsive'}")
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
    print("  Enter angle 0–180 to pan (rover rotates via motors), 'tilt <angle>' for tilt.")
    print("  Pan 90 = face forward.  Ensure clear space.  'q' to continue.\n")
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
    print(f"  Pan axis  : SERVO direction={PAN_DIR}  (LEFT) — motor tank-turn")
    print(f"  Tilt axis : SERVO direction={TILT_DIR}  (UP)  — physical servo")
    print(f"  IMU verify: {'ON' if VERIFY_WITH_IMU else 'OFF'}  "
          f"(tolerance ±{IMU_TOLERANCE_DEG}°)\n")

    # ── Optional IMU init ─────────────────────────────────────────────────────
    imu_bus = None
    if VERIFY_WITH_IMU:
        try:
            import smbus2  # type: ignore
            imu_bus = smbus2.SMBus(I2C_BUS)
            if _init_imu(imu_bus):
                h = _read_heading(imu_bus)
                print(f"IMU connected — initial heading {h:.1f}°" if h else
                      "IMU connected but no heading reading yet")
            else:
                print("IMU init failed — pan IMU verification disabled")
                imu_bus.close()
                imu_bus = None
        except ImportError:
            print("smbus2 not installed — pan IMU verification disabled")
        except Exception as e:
            print(f"IMU error ({e}) — pan IMU verification disabled")
            imu_bus = None
    print()

    print("Connecting to Arduino…")
    ser = _open_and_handshake()
    if ser is None:
        print("Could not connect — exiting.")
        raise SystemExit(1)
    print("Handshake complete.\n")

    # Return to known starting position before any tests
    print("Homing both axes to centre (90°)…")
    _home(ser, imu_bus)
    time.sleep(0.5)

    results: list[tuple[str, bool]] = []

    # ── Section 1: Pan axis ───────────────────────────────────────────────────
    print("\n=== PAN (motor tank-turn + IMU verify) ===")
    print("The rover physically rotates — ensure 180° of clear space around it.\n")
    for fn in [
        test_pan_centre,
        test_pan_full_left,
        test_pan_full_right,
        test_pan_sweep_manual,
    ]:
        results.append((fn.__name__, fn(ser, imu_bus)))
        _home(ser, imu_bus)

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
        _home(ser, imu_bus)

    # ── Section 3: Combined axis ──────────────────────────────────────────────
    print("\n=== COMBINED AXIS ===\n")
    for fn in [test_simultaneous_move, test_rapid_commands]:
        results.append((fn.__name__, fn(ser)))
        _home(ser, imu_bus)

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
        _home(ser, imu_bus)

    # ── Section 5: Edge cases ─────────────────────────────────────────────────
    print("\n=== EDGE CASES ===\n")
    for fn in [
        test_servo_clamp_below_zero,
        test_servo_clamp_above_one,
        test_servo_speed_field_ignored,
    ]:
        results.append((fn.__name__, fn(ser)))
        _home(ser, imu_bus)

    # ── Interactive check (optional) ──────────────────────────────────────────
    print("\n=== INTERACTIVE CHECK (optional) ===")
    answer = input("Run interactive servo control? [y/N]: ").strip().lower()
    if answer == "y":
        _visual_check(ser)

    # ── Return to home and close ──────────────────────────────────────────────
    print("Returning servos to centre…")
    _home(ser, imu_bus)
    ser.close()
    if imu_bus is not None:
        imu_bus.close()

    # ── Results summary ───────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("SERVO TEST RESULTS")
    print("=" * 60)
    passed_n = sum(1 for _, p in results if p)
    failed_n = sum(1 for _, p in results if not p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n  {passed_n} passed  |  {failed_n} failed  |  {len(results)} total")

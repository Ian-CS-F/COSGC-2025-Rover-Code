"""
Test Protocol: Comprehensive Serial Communications
Tests every command and response type exactly as used in main.py / rover_interpreter.ino.

Command format:  (<system>, <direction>, <amount>, <speed>)\n
Systems:         0=SERVO  1=MOTOR  2=SWEEP  3=QUERY  4=FLIP
Directions:      0=LEFT   1=RIGHT  2=UP     3=DOWN
                 (LEFT/RIGHT always run motors continuously and never send DONE)

All tests share ONE serial connection opened at the top — re-opening resets
the Arduino, which takes ~10 s to re-handshake.

Requires: Arduino connected, running rover_interpreter.ino.
Place the rover on a stand (wheels off ground) for all motor tests.
"""

import time
import serial

# ── Constants from main.py ────────────────────────────────────────────────────
SERIAL_PORT        = "/dev/ttyACM0"
BAUD_RATE          = 9600
HANDSHAKE_TIMEOUT  = 10
MOTOR_DONE_TIMEOUT = 30.0

SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT,  RIGHT, UP,   DOWN         = 0, 1, 2, 3

# Tilt angles the Arduino sweeps through (TILT_ANGLES[] in rover_interpreter.ino)
# Reported in AT: messages as offset from 90° → [-30, -15, 0, +15, +30]
VALID_TILT_OFFSETS = {-30, -15, 0, 15, 30}

# ── Shared helpers ────────────────────────────────────────────────────────────
def send_command(ser, system, direction, amount, speed):
    """Send a (System,Direction,Amount,Speed) packet — exact format from main.py."""
    packet = f"({system},{direction},{amount:.3f},{speed:.3f})\n"
    ser.write(packet.encode())

def readline(ser, timeout=3.0):
    """Read one line within timeout. Returns stripped string or '' on timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            return line
    return ""

def wait_for(ser, prefix, timeout=5.0):
    """Read lines until one starts with prefix, or timeout. Returns the line."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    return ""

def handshake(ser):
    """Complete READY/ACK handshake. Returns True on success."""
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            return True
    return False

def stop_motors(ser):
    send_command(ser, MOTOR, UP, 0.0, 0.0)

# ── Section 1: Connection ──────────────────────────────────────────────────────
def test_port_opens():
    """Serial port opens at the correct baud rate without error."""
    print("TEST: Port opens at 9600 baud...")
    try:
        s = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        s.close()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL — {e}")
        return False

def test_ready_received(ser):
    """Arduino sends 'READY' immediately (within 10 s of connection)."""
    print("TEST: Arduino sends READY...")
    # Already waiting inside the open connection — flush and check
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            print("  PASS")
            return True
    print("  FAIL — READY not received")
    return False

def test_ack_completes_handshake(ser):
    """Sending 'ACK' after READY stops the READY loop and unlocks commands."""
    print("TEST: ACK completes handshake...")
    ser.write(b"ACK\n")
    # Give the Arduino a moment, then confirm it is NOT still sending READY
    time.sleep(1.5)
    ser.reset_input_buffer()  # discard anything still in flight
    # If we can send a QUERY and get DIST back, the handshake completed
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS' if passed else 'FAIL — no DIST response after handshake'}")
    return passed

# ── Section 2: Packet format ──────────────────────────────────────────────────
def test_packet_format_accepted(ser):
    """A correctly formatted packet is parsed and executed by the Arduino."""
    print("TEST: Correct packet format is parsed and executed...")
    send_command(ser, QUERY, 0, 0.0, 0.0)  # (3,0,0.000,0.000)\n
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  Sent: (3,0,0.000,0.000)  Got: '{resp}'  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_malformed_packet_ignored(ser):
    """A packet without parentheses is silently ignored (no response, no crash)."""
    print("TEST: Malformed packet is ignored...")
    ser.write(b"3,0,0.000,0.000\n")   # missing parens — parsePacket returns false
    time.sleep(0.5)
    # Now send a valid QUERY — if we still get DIST:, the Arduino is alive
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS — Arduino still responsive after bad packet' if passed else 'FAIL — Arduino appears hung'}")
    return passed

def test_incomplete_packet_ignored(ser):
    """A packet with missing fields is silently ignored."""
    print("TEST: Incomplete packet (missing fields) is ignored...")
    ser.write(b"(3,0)\n")  # only 2 fields, parsePacket returns false
    time.sleep(0.5)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS — Arduino still responsive' if passed else 'FAIL'}")
    return passed

# ── Section 3: QUERY (system 3) ───────────────────────────────────────────────
def test_query_returns_dist(ser):
    """QUERY returns 'DIST:<float>' immediately."""
    print("TEST: QUERY returns DIST:<cm>...")
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    try:
        float(resp[5:])
        passed = True
    except (ValueError, IndexError):
        passed = False
    print(f"  Response: '{resp}'  →  {'PASS' if passed else 'FAIL (not a valid float)'}")
    return passed

def test_query_resets_encoder(ser):
    """Two back-to-back QUERYs with no movement: second should return 0.0."""
    print("TEST: QUERY resets encoder (second query with no movement = 0.0)...")
    # First query — drains whatever is accumulated
    send_command(ser, QUERY, 0, 0.0, 0.0)
    wait_for(ser, "DIST:", timeout=3.0)
    # Second query immediately after — no movement → should be 0.0
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    try:
        dist = float(resp[5:])
        passed = abs(dist) < 0.1  # encoder count resets to 0 after first query
    except (ValueError, IndexError):
        passed = False
    print(f"  Second DIST: {resp}  →  {'PASS' if passed else 'FAIL (expected ~0.0)'}")
    return passed

def test_query_sequential(ser):
    """Three sequential QUERY commands each return a separate DIST: line (no cross-talk)."""
    print("TEST: Three sequential QUERYs each return their own DIST:...")
    results = []
    for _ in range(3):
        send_command(ser, QUERY, 0, 0.0, 0.0)
        resp = wait_for(ser, "DIST:", timeout=3.0)
        results.append(resp)
    passed = all(r.startswith("DIST:") for r in results)
    print(f"  Responses: {results}  →  {'PASS' if passed else 'FAIL (missing or wrong responses)'}")
    return passed

# ── Section 4: SERVO (system 0) ───────────────────────────────────────────────
def test_servo_centre(ser):
    """SERVO to 0.5 (90°) — no crash, Arduino remains responsive."""
    print("TEST: SERVO 0.5 (90°) — no response expected, Arduino stays alive...")
    send_command(ser, SERVO, UP, 0.5, 0.0)   # direction=UP, amount=0.5 → 90°
    time.sleep(0.3)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive after SERVO command'}")
    return passed

def test_servo_full_range(ser):
    """SERVO sweeps from 0.0 (0°) to 1.0 (180°) and back — Arduino stays alive."""
    print("TEST: SERVO 0.0→1.0→0.5 (0°→180°→90°) — Arduino stays alive...")
    for amount in (0.0, 1.0, 0.5):
        send_command(ser, SERVO, UP, amount, 0.0)
        time.sleep(0.3)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Section 5: MOTOR UP/DOWN → DONE (system 1) ────────────────────────────────
def test_motor_up_done_format(ser):
    """
    MOTOR UP 5 cm → DONE:<actual_cm>,<error_ratio>
    Verifies the response arrives and both fields parse as floats.
    """
    print("TEST: MOTOR UP 5 cm → DONE:<cm>,<ratio> format...")
    send_command(ser, MOTOR, UP, 5.0, 0.7)
    resp = wait_for(ser, "DONE:", timeout=MOTOR_DONE_TIMEOUT)
    try:
        parts = resp[5:].split(",")
        actual_cm   = float(parts[0])
        error_ratio = float(parts[1])
        passed = actual_cm >= 0.0 and error_ratio > 0.0
    except (ValueError, IndexError):
        passed = False
        actual_cm = error_ratio = None
    print(f"  Response: '{resp}'  actual={actual_cm}  ratio={error_ratio}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_motor_down_done_format(ser):
    """MOTOR DOWN 5 cm → DONE:<cm>,<ratio>."""
    print("TEST: MOTOR DOWN 5 cm → DONE:<cm>,<ratio>...")
    send_command(ser, MOTOR, DOWN, 5.0, 0.7)
    resp = wait_for(ser, "DONE:", timeout=MOTOR_DONE_TIMEOUT)
    try:
        parts = resp[5:].split(",")
        actual_cm   = float(parts[0])
        error_ratio = float(parts[1])
        passed = actual_cm >= 0.0 and error_ratio > 0.0
    except (ValueError, IndexError):
        passed = False
        actual_cm = error_ratio = None
    print(f"  Response: '{resp}'  actual={actual_cm}  ratio={error_ratio}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_motor_done_encoder_nonzero(ser):
    """DONE actual_cm should be > 0 when wheels spin (encoder is connected)."""
    print("TEST: DONE actual_cm > 0 (encoder counts increment)...")
    send_command(ser, MOTOR, UP, 10.0, 0.7)
    resp = wait_for(ser, "DONE:", timeout=MOTOR_DONE_TIMEOUT)
    try:
        actual_cm = float(resp[5:].split(",")[0])
        passed = actual_cm > 0.0
    except (ValueError, IndexError):
        passed = False
        actual_cm = None
    print(f"  actual_cm = {actual_cm}  →  {'PASS' if passed else 'FAIL (encoder may not be wired)'}")
    return passed

def test_motor_done_error_ratio_near_one(ser):
    """
    error_ratio = actual_counts / target_counts.
    On a stand with no load, slipping is unlikely — ratio should be 0.5–1.5.
    """
    print("TEST: DONE error_ratio is plausible (0.5–1.5) when wheels spin freely...")
    send_command(ser, MOTOR, UP, 10.0, 0.7)
    resp = wait_for(ser, "DONE:", timeout=MOTOR_DONE_TIMEOUT)
    try:
        error_ratio = float(resp[5:].split(",")[1])
        passed = 0.5 <= error_ratio <= 1.5
    except (ValueError, IndexError):
        passed = False
        error_ratio = None
    print(f"  error_ratio = {error_ratio}  →  {'PASS' if passed else 'FAIL (outside 0.5–1.5)'}")
    return passed

def test_motor_continuous_no_done(ser):
    """
    MOTOR UP with distance=0 (continuous) never sends DONE.
    After 1 s, stop with speed=0. Confirm no DONE arrived.
    """
    print("TEST: MOTOR continuous (distance=0) sends no DONE...")
    ser.reset_input_buffer()
    send_command(ser, MOTOR, UP, 0.0, 0.5)
    time.sleep(1.0)
    stop_motors(ser)
    time.sleep(0.2)
    # Drain any buffered lines and check none are DONE
    done_received = False
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DONE:"):
            done_received = True
    passed = not done_received
    print(f"  {'PASS — no DONE received in continuous mode' if passed else 'FAIL — unexpected DONE'}")
    return passed

def test_motor_continuous_encoder_moves(ser):
    """
    MOTOR UP continuous for 1 s → stop → QUERY confirms encoder moved.
    """
    print("TEST: MOTOR continuous for 1 s — encoder reports movement via QUERY...")
    send_command(ser, QUERY, 0, 0.0, 0.0)   # drain encoder count
    wait_for(ser, "DIST:", timeout=3.0)
    send_command(ser, MOTOR, UP, 0.0, 0.5)
    time.sleep(1.0)
    stop_motors(ser)
    time.sleep(0.1)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    try:
        dist = float(resp[5:])
        passed = abs(dist) > 0.0
    except (ValueError, IndexError):
        passed = False
        dist = None
    print(f"  DIST after 1 s drive: {dist} cm  →  {'PASS' if passed else 'FAIL (encoder did not move)'}")
    return passed

# ── Section 6: MOTOR LEFT / RIGHT (always continuous, never DONE) ─────────────
def test_motor_left_no_done(ser):
    """
    LEFT always returns immediately in the Arduino (setMotor + return).
    DONE is never sent regardless of amount value.
    """
    print("TEST: MOTOR LEFT — no DONE ever (runs until stopped)...")
    ser.reset_input_buffer()
    send_command(ser, MOTOR, LEFT, 0.0, 0.5)
    time.sleep(1.0)
    stop_motors(ser)
    time.sleep(0.2)
    done_received = any(
        ser.readline().decode(errors="replace").strip().startswith("DONE:")
        for _ in range(10) if ser.in_waiting
    )
    passed = not done_received
    print(f"  {'PASS — no DONE for LEFT' if passed else 'FAIL — unexpected DONE'}")
    return passed

def test_motor_left_encoder_moves(ser):
    """MOTOR LEFT 1 s → stop → QUERY encoder moved."""
    print("TEST: MOTOR LEFT 1 s — encoder moves...")
    send_command(ser, QUERY, 0, 0.0, 0.0)
    wait_for(ser, "DIST:", timeout=3.0)
    send_command(ser, MOTOR, LEFT, 0.0, 0.5)
    time.sleep(1.0)
    stop_motors(ser)
    time.sleep(0.1)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    try:
        dist = float(resp[5:])
        passed = abs(dist) > 0.0
    except (ValueError, IndexError):
        passed = False
        dist = None
    print(f"  DIST after LEFT 1 s: {dist} cm  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_motor_right_encoder_moves(ser):
    """MOTOR RIGHT 1 s → stop → QUERY encoder moved."""
    print("TEST: MOTOR RIGHT 1 s — encoder moves...")
    send_command(ser, QUERY, 0, 0.0, 0.0)
    wait_for(ser, "DIST:", timeout=3.0)
    send_command(ser, MOTOR, RIGHT, 0.0, 0.5)
    time.sleep(1.0)
    stop_motors(ser)
    time.sleep(0.1)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    try:
        dist = float(resp[5:])
        passed = abs(dist) > 0.0
    except (ValueError, IndexError):
        passed = False
        dist = None
    print(f"  DIST after RIGHT 1 s: {dist} cm  →  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Section 7: FLIP (system 4) ────────────────────────────────────────────────
def test_flip_on_no_crash(ser):
    """FLIP=1.0 (inverted mode) — no response, Arduino stays alive."""
    print("TEST: FLIP on (1.0) — no response, Arduino stays alive...")
    send_command(ser, FLIP, 0, 1.0, 0.0)
    time.sleep(0.2)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS' if passed else 'FAIL — Arduino unresponsive after FLIP on'}")
    return passed

def test_flip_off_restores_normal(ser):
    """FLIP=0.0 (normal mode) — motors should behave normally again."""
    print("TEST: FLIP off (0.0) — normal mode restored, QUERY responds...")
    send_command(ser, FLIP, 0, 0.0, 0.0)
    time.sleep(0.2)
    send_command(ser, QUERY, 0, 0.0, 0.0)
    resp = wait_for(ser, "DIST:", timeout=3.0)
    passed = resp.startswith("DIST:")
    print(f"  {'PASS' if passed else 'FAIL'}")
    return passed

def test_flip_inverts_motor_direction(ser):
    """
    With FLIP=1 active, UP command should drive wheels backward.
    Confirm encoder still moves (motor is driven), just in the opposite direction.
    """
    print("TEST: FLIP=1 — UP command inverts motor direction (encoder still moves)...")
    send_command(ser, FLIP, 0, 1.0, 0.0)   # enter flipped mode
    time.sleep(0.1)
    send_command(ser, QUERY, 0, 0.0, 0.0)  # reset encoder
    wait_for(ser, "DIST:", timeout=3.0)
    send_command(ser, MOTOR, UP, 5.0, 0.7)
    resp = wait_for(ser, "DONE:", timeout=MOTOR_DONE_TIMEOUT)
    send_command(ser, FLIP, 0, 0.0, 0.0)   # restore normal mode
    try:
        actual_cm = float(resp[5:].split(",")[0])
        passed = actual_cm > 0.0  # wheels still turned, just backward
    except (ValueError, IndexError):
        passed = False
        actual_cm = None
    print(f"  actual_cm (flipped UP) = {actual_cm}  →  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Section 8: SWEEP (system 2) ───────────────────────────────────────────────
def test_sweep_sends_at_messages(ser):
    """
    SWEEP 10° range — Arduino should send at least one AT:<h>,<t> message
    before SWEEP_DONE.
    """
    print("TEST: SWEEP 10° — AT: messages received before SWEEP_DONE...")
    send_command(ser, SWEEP, 0, 10.0, 100.0)  # 10° range, 100 ms/step
    at_count = 0
    sweep_done = False
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        line = readline(ser, timeout=5.0)
        if line.startswith("AT:"):
            at_count += 1
            ser.write(b"NEXT\n")
        elif line == "SWEEP_DONE":
            sweep_done = True
            break
    passed = at_count > 0 and sweep_done
    print(f"  AT messages: {at_count}  SWEEP_DONE: {sweep_done}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_sweep_at_message_format(ser):
    """
    AT:<h_angle>,<tilt_offset> — both fields must parse as numbers.
    h_angle is the horizontal position (float), tilt_offset is tilt - 90 (int).
    """
    print("TEST: SWEEP 10° — AT: messages have valid numeric format...")
    send_command(ser, SWEEP, 0, 10.0, 100.0)
    at_valid = []
    sweep_done = False
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        line = readline(ser, timeout=5.0)
        if line.startswith("AT:"):
            try:
                parts = line[3:].split(",")
                float(parts[0])   # h_angle
                int(parts[1])     # tilt offset
                at_valid.append(True)
            except (ValueError, IndexError):
                at_valid.append(False)
            ser.write(b"NEXT\n")
        elif line == "SWEEP_DONE":
            sweep_done = True
            break
    passed = len(at_valid) > 0 and all(at_valid) and sweep_done
    print(f"  Valid AT msgs: {sum(at_valid)}/{len(at_valid)}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_sweep_tilt_angles_valid(ser):
    """
    Tilt offsets in AT: messages must come from TILT_ANGLES[] - 90:
    valid values are {-30, -15, 0, +15, +30}.
    """
    print("TEST: SWEEP 10° — tilt offsets are only from {-30,-15,0,+15,+30}...")
    send_command(ser, SWEEP, 0, 10.0, 100.0)
    bad_tilts = []
    sweep_done = False
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        line = readline(ser, timeout=5.0)
        if line.startswith("AT:"):
            try:
                tilt_offset = int(line[3:].split(",")[1])
                if tilt_offset not in VALID_TILT_OFFSETS:
                    bad_tilts.append(tilt_offset)
            except (ValueError, IndexError):
                bad_tilts.append("parse error")
            ser.write(b"NEXT\n")
        elif line == "SWEEP_DONE":
            sweep_done = True
            break
    passed = len(bad_tilts) == 0 and sweep_done
    print(f"  Bad tilt offsets: {bad_tilts}  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_sweep_ends_with_sweep_done(ser):
    """SWEEP always terminates with exactly 'SWEEP_DONE' (no extra chars)."""
    print("TEST: SWEEP 10° — terminates with exactly 'SWEEP_DONE'...")
    send_command(ser, SWEEP, 0, 10.0, 100.0)
    last_line = ""
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        line = readline(ser, timeout=5.0)
        if not line:
            break
        if line.startswith("AT:"):
            ser.write(b"NEXT\n")
        if line == "SWEEP_DONE":
            last_line = line
            break
    passed = last_line == "SWEEP_DONE"
    print(f"  Final message: '{last_line}'  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_sweep_next_required(ser):
    """
    Without sending NEXT, Arduino waits at each AT: position.
    After a delay, send NEXT and verify the sweep still progresses.
    """
    print("TEST: SWEEP — Arduino waits at AT: until NEXT is sent...")
    send_command(ser, SWEEP, 0, 10.0, 200.0)
    # Get first AT: without responding
    first_at = wait_for(ser, "AT:", timeout=10.0)
    if not first_at:
        print("  FAIL — no AT: received")
        return False
    # Wait 300 ms without sending NEXT — Arduino should be blocked
    time.sleep(0.3)
    # Now send NEXT and confirm we get another AT: (or SWEEP_DONE)
    ser.write(b"NEXT\n")
    second = readline(ser, timeout=5.0)
    # Drain the rest of the sweep
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        line = readline(ser, timeout=5.0)
        if not line:
            break
        if line.startswith("AT:"):
            ser.write(b"NEXT\n")
        if line == "SWEEP_DONE":
            break
    passed = second.startswith("AT:") or second == "SWEEP_DONE"
    print(f"  After NEXT, received: '{second}'  →  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Section 9: CLIFF message format ───────────────────────────────────────────
def test_cliff_format_if_triggered(ser):
    """
    If the rover is lifted off the ground during a short drive, the Arduino
    sends CLIFF:<float>. This test asks the operator to lift the rover and
    verifies the message format.

    Safe to skip — press Enter without lifting to skip.
    """
    print("TEST: CLIFF:<cm> format — lift the rover now to trigger (Enter to skip)...")
    answer = input("  Lift rover off ground? (y to test, Enter to skip): ").strip().lower()
    if answer != "y":
        print("  SKIP")
        return None

    send_command(ser, MOTOR, UP, 0.0, 0.5)  # continuous drive
    resp = wait_for(ser, "CLIFF:", timeout=10.0)
    stop_motors(ser)

    if not resp:
        print("  FAIL — no CLIFF: received (was the rover actually lifted?)")
        return False
    try:
        dist_cm = float(resp[6:])
        passed = dist_cm >= 0.0
    except ValueError:
        passed = False
        dist_cm = None
    print(f"  CLIFF: response: '{resp}'  dist={dist_cm} cm  →  {'PASS' if passed else 'FAIL'}")
    return passed

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("NOTE: Place rover on a stand (wheels off ground) before running.\n")

    # ── Open port and handshake ────────────────────────────────────────────────
    results = []

    port_ok = test_port_opens()
    results.append(("test_port_opens", port_ok))
    if not port_ok:
        print("\nCannot open serial port — all subsequent tests skipped.")
        exit(1)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()

    ready_ok = test_ready_received(ser)
    results.append(("test_ready_received", ready_ok))
    if not ready_ok:
        print("\nArduino did not send READY — all subsequent tests skipped.")
        ser.close()
        exit(1)

    ack_ok = test_ack_completes_handshake(ser)
    results.append(("test_ack_completes_handshake", ack_ok))
    if not ack_ok:
        print("\nHandshake failed — all subsequent tests skipped.")
        ser.close()
        exit(1)

    print("\n=== PACKET FORMAT ===")
    for t in [test_packet_format_accepted, test_malformed_packet_ignored, test_incomplete_packet_ignored]:
        results.append((t.__name__, t(ser)))

    print("\n=== QUERY (system 3) ===")
    for t in [test_query_returns_dist, test_query_resets_encoder, test_query_sequential]:
        results.append((t.__name__, t(ser)))

    print("\n=== SERVO (system 0) ===")
    for t in [test_servo_centre, test_servo_full_range]:
        results.append((t.__name__, t(ser)))

    print("\n=== MOTOR UP/DOWN → DONE (system 1) ===")
    for t in [
        test_motor_up_done_format,
        test_motor_down_done_format,
        test_motor_done_encoder_nonzero,
        test_motor_done_error_ratio_near_one,
        test_motor_continuous_no_done,
        test_motor_continuous_encoder_moves,
    ]:
        results.append((t.__name__, t(ser)))

    print("\n=== MOTOR LEFT / RIGHT (no DONE) ===")
    for t in [test_motor_left_no_done, test_motor_left_encoder_moves, test_motor_right_encoder_moves]:
        results.append((t.__name__, t(ser)))

    print("\n=== FLIP (system 4) ===")
    for t in [test_flip_on_no_crash, test_flip_off_restores_normal, test_flip_inverts_motor_direction]:
        results.append((t.__name__, t(ser)))

    print("\n=== SWEEP (system 2) ===")
    for t in [
        test_sweep_sends_at_messages,
        test_sweep_at_message_format,
        test_sweep_tilt_angles_valid,
        test_sweep_ends_with_sweep_done,
        test_sweep_next_required,
    ]:
        results.append((t.__name__, t(ser)))

    print("\n=== CLIFF (optional) ===")
    cliff_result = test_cliff_format_if_triggered(ser)
    if cliff_result is not None:
        results.append(("test_cliff_format_if_triggered", cliff_result))

    ser.close()

    print("\n" + "=" * 55)
    print("SERIAL COMMS TEST RESULTS")
    print("=" * 55)
    passed_n  = sum(1 for _, p in results if p is True)
    failed_n  = sum(1 for _, p in results if p is False)
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  {status}  {name}")
    print(f"\n  {passed_n} passed  |  {failed_n} failed  |  {len(results)} total")

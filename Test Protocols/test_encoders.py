"""
Test Protocol: Encoders (no motor movement)
============================================
Tests both wheel encoders by having the operator spin each wheel by hand.
No motor commands are sent — motors stay completely off throughout.

The only Arduino command used is QUERY (system 3):
    Pi sends    (3, 0, 0.000, 0.000)
    Arduino replies  DIST:<left_cm>,<right_cm>  and resets both counters.

Encoder hardware (from rover_interpreter.ino):
    Left  encoder — interrupt pin 2 (INT0), direction pin 10
    Right encoder — interrupt pin 3 (INT1), direction pin 11
    COUNTS_PER_REV   = 64      (rising edges on channel A only)
    DIST_PER_REV_CM  = 10.0    one full revolution ≈ 10 cm reported

Tests:
    1. Baseline    — two back-to-back QUERYs with no movement give 0.0
    2. Left fwd    — spin left wheel forward → DIST left > 0
    3. Left back   — spin left wheel backward → DIST left < 0
    4. Right fwd   — spin right wheel forward → DIST right > 0
    5. Right back  — spin right wheel backward → DIST right < 0
    6. Independence — spin only left wheel → right reads ≈ 0 (and vice versa)
    7. Resolution  — one full revolution ≈ 10 cm (within ±30 % tolerance)

Run on the Pi:
    python3 "test_encoders.py"
"""

import time
import serial

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyACM0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10    # seconds
QUERY_TIMEOUT     = 3.0   # seconds to wait for DIST:

# From rover_interpreter.ino — used to check resolution
COUNTS_PER_REV  = 64
DIST_PER_REV_CM = 10.0   # cm per full revolution

# Independence check: the un-spun wheel must read less than this
IDLE_TOLERANCE_CM = 0.5

# Resolution check: one full revolution should land within this fraction of expected
RESOLUTION_TOLERANCE = 0.30   # ±30 %

# ── Serial helpers ────────────────────────────────────────────────────────────
def _open_and_handshake() -> "serial.Serial | None":
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


def _query(ser: serial.Serial) -> tuple[float, float] | None:
    """
    Send QUERY and return (left_cm, right_cm), or None on timeout.
    Also resets the Arduino's encoder counters.
    """
    ser.write(b"(3,0,0.000,0.000)\n")
    deadline = time.monotonic() + QUERY_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            try:
                parts = line[5:].split(",")
                return float(parts[0]), float(parts[1]) if len(parts) > 1 else float(parts[0])
            except (ValueError, IndexError):
                return None
    return None


def _drain(ser: serial.Serial) -> None:
    """Send QUERY and discard the result — zeroes the counters cleanly."""
    _query(ser)


def _prompt_spin(wheel: str, direction: str, extra: str = "") -> None:
    """Print a clear instruction and wait for the operator to press Enter."""
    print(f"\n  >>> Spin the {wheel.upper()} wheel {direction.upper()} by hand{extra}.")
    input("      Press Enter when done: ")


# ═════════════════════════════════════════════════════════════════════════════
# Tests
# ═════════════════════════════════════════════════════════════════════════════

def test_query_format(ser: serial.Serial) -> bool:
    """
    QUERY must return a line matching DIST:<float>,<float>.
    Verifies the protocol is working before any movement tests.
    """
    print("TEST: QUERY returns DIST:<left_cm>,<right_cm> format…")
    result = _query(ser)
    passed = result is not None
    if passed:
        left_cm, right_cm = result
        print(f"  DIST: left={left_cm:.4f} cm  right={right_cm:.4f} cm  →  PASS")
    else:
        print("  FAIL — no DIST: response within timeout")
    return passed


def test_baseline_zero(ser: serial.Serial) -> bool:
    """
    Two consecutive QUERYs with no wheel movement between them.
    The second must return 0.0 on both sides (counters reset by first QUERY).
    """
    print("TEST: Baseline — second QUERY with no movement returns 0.0…")
    _drain(ser)                    # first query — resets counters
    time.sleep(0.1)
    result = _query(ser)           # second query — should read 0
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, right_cm = result
    passed = abs(left_cm) < IDLE_TOLERANCE_CM and abs(right_cm) < IDLE_TOLERANCE_CM
    print(f"  left={left_cm:.4f} cm  right={right_cm:.4f} cm  "
          f"(both should be ~0)  →  {'PASS' if passed else 'FAIL'}")
    return passed


def test_left_encoder_forward(ser: serial.Serial) -> bool:
    """
    Spin the LEFT wheel forward (in the normal drive direction).
    DIST left_cm must be positive (encoder counts increase).
    MOTORS STAY OFF — spin the wheel by hand.
    """
    print("TEST: Left encoder — forward spin gives positive distance…")
    _drain(ser)
    _prompt_spin("left", "FORWARD", " (a few turns)")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, _ = result
    passed = left_cm > 0.0
    print(f"  left_cm = {left_cm:.3f} cm  →  {'PASS' if passed else 'FAIL (expected > 0)'}")
    return passed


def test_left_encoder_backward(ser: serial.Serial) -> bool:
    """
    Spin the LEFT wheel backward.
    DIST left_cm must be negative (encoder counts decrease).
    """
    print("TEST: Left encoder — backward spin gives negative distance…")
    _drain(ser)
    _prompt_spin("left", "BACKWARD", " (a few turns)")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, _ = result
    passed = left_cm < 0.0
    print(f"  left_cm = {left_cm:.3f} cm  →  {'PASS' if passed else 'FAIL (expected < 0)'}")
    return passed


def test_right_encoder_forward(ser: serial.Serial) -> bool:
    """
    Spin the RIGHT wheel forward.
    DIST right_cm must be positive.
    """
    print("TEST: Right encoder — forward spin gives positive distance…")
    _drain(ser)
    _prompt_spin("right", "FORWARD", " (a few turns)")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    _, right_cm = result
    passed = right_cm > 0.0
    print(f"  right_cm = {right_cm:.3f} cm  →  {'PASS' if passed else 'FAIL (expected > 0)'}")
    return passed


def test_right_encoder_backward(ser: serial.Serial) -> bool:
    """
    Spin the RIGHT wheel backward.
    DIST right_cm must be negative.
    """
    print("TEST: Right encoder — backward spin gives negative distance…")
    _drain(ser)
    _prompt_spin("right", "BACKWARD", " (a few turns)")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    _, right_cm = result
    passed = right_cm < 0.0
    print(f"  right_cm = {right_cm:.3f} cm  →  {'PASS' if passed else 'FAIL (expected < 0)'}")
    return passed


def test_independence_left_only(ser: serial.Serial) -> bool:
    """
    Spin ONLY the left wheel forward.
    right_cm must stay within ±IDLE_TOLERANCE_CM — confirms the encoders
    are electrically isolated and channel B wiring is correct.
    """
    print("TEST: Independence — only left spins → right reads ≈ 0…")
    _drain(ser)
    _prompt_spin("LEFT ONLY", "FORWARD",
                 f" — do NOT touch the right wheel")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, right_cm = result
    passed = left_cm > 0.0 and abs(right_cm) < IDLE_TOLERANCE_CM
    print(f"  left={left_cm:.3f} cm (should be >0)  "
          f"right={right_cm:.3f} cm (should be ~0, limit ±{IDLE_TOLERANCE_CM})  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_independence_right_only(ser: serial.Serial) -> bool:
    """
    Spin ONLY the right wheel forward.
    left_cm must stay within ±IDLE_TOLERANCE_CM.
    """
    print("TEST: Independence — only right spins → left reads ≈ 0…")
    _drain(ser)
    _prompt_spin("RIGHT ONLY", "FORWARD",
                 f" — do NOT touch the left wheel")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, right_cm = result
    passed = right_cm > 0.0 and abs(left_cm) < IDLE_TOLERANCE_CM
    print(f"  left={left_cm:.3f} cm (should be ~0, limit ±{IDLE_TOLERANCE_CM})  "
          f"right={right_cm:.3f} cm (should be >0)  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_resolution_left(ser: serial.Serial) -> bool:
    """
    Spin the LEFT wheel forward exactly ONE full revolution.
    Reported distance should be within ±RESOLUTION_TOLERANCE of DIST_PER_REV_CM
    (nominally 10.0 cm, tolerance ±30 %).

    Formula:  DIST_PER_REV_CM = COUNTS_PER_REV / COUNTS_PER_REV * DIST_PER_REV_CM
              One rev → {COUNTS_PER_REV} counts → {DIST_PER_REV_CM} cm reported.
    """
    lo = DIST_PER_REV_CM * (1 - RESOLUTION_TOLERANCE)
    hi = DIST_PER_REV_CM * (1 + RESOLUTION_TOLERANCE)
    print(f"TEST: Resolution — 1 full left revolution should report "
          f"{lo:.1f}–{hi:.1f} cm…")
    print(f"  (COUNTS_PER_REV={COUNTS_PER_REV}, DIST_PER_REV_CM={DIST_PER_REV_CM})")
    _drain(ser)
    print(f"\n  >>> Spin the LEFT wheel forward EXACTLY ONE full revolution.")
    input("      Line up a mark before and after.  Press Enter when done: ")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, _ = result
    passed = lo <= left_cm <= hi
    print(f"  Reported: {left_cm:.3f} cm  Expected: {DIST_PER_REV_CM:.1f} ±{RESOLUTION_TOLERANCE*100:.0f}%  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_resolution_right(ser: serial.Serial) -> bool:
    """
    Same resolution check for the RIGHT wheel.
    One full revolution forward should report within ±30 % of 10.0 cm.
    """
    lo = DIST_PER_REV_CM * (1 - RESOLUTION_TOLERANCE)
    hi = DIST_PER_REV_CM * (1 + RESOLUTION_TOLERANCE)
    print(f"TEST: Resolution — 1 full right revolution should report "
          f"{lo:.1f}–{hi:.1f} cm…")
    _drain(ser)
    print(f"\n  >>> Spin the RIGHT wheel forward EXACTLY ONE full revolution.")
    input("      Line up a mark before and after.  Press Enter when done: ")
    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    _, right_cm = result
    passed = lo <= right_cm <= hi
    print(f"  Reported: {right_cm:.3f} cm  Expected: {DIST_PER_REV_CM:.1f} ±{RESOLUTION_TOLERANCE*100:.0f}%  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_query_accumulates(ser: serial.Serial) -> bool:
    """
    Spin the left wheel, do NOT query, spin it again, then query.
    The reported distance must reflect both spins (counts accumulate between queries).
    """
    print("TEST: Encoder accumulates counts between queries…")
    _drain(ser)

    print("\n  >>> Spin the LEFT wheel forward a few turns.")
    input("      Press Enter when done (do NOT let go yet): ")
    print("  (do NOT query yet — spinning again now)")
    print("\n  >>> Spin the LEFT wheel forward a few MORE turns.")
    input("      Press Enter when done: ")

    result = _query(ser)
    if result is None:
        print("  FAIL — no DIST: response")
        return False
    left_cm, _ = result
    # Should be non-trivially positive — more than one spin's worth
    passed = left_cm > DIST_PER_REV_CM * 0.5
    print(f"  Accumulated left_cm = {left_cm:.3f} cm  "
          f"(should be well above {DIST_PER_REV_CM * 0.5:.1f})  "
          f"→  {'PASS' if passed else 'FAIL (counts may have reset early)'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  COSGC Rover — Encoder Test (no motor movement)")
    print("=" * 60)
    print(f"  Port            : {SERIAL_PORT}  @  {BAUD_RATE} baud")
    print(f"  Counts/rev      : {COUNTS_PER_REV}")
    print(f"  Dist/rev        : {DIST_PER_REV_CM} cm")
    print(f"  Idle tolerance  : ±{IDLE_TOLERANCE_CM} cm")
    print(f"  Resolution tol  : ±{RESOLUTION_TOLERANCE*100:.0f}%")
    print()
    print("  Motors will NOT move.  Spin wheels by hand when prompted.")
    print("=" * 60)

    print("\nConnecting to Arduino…")
    ser = _open_and_handshake()
    if ser is None:
        raise SystemExit(1)
    print("Handshake complete.\n")

    results: list[tuple[str, bool]] = []

    print("=== PROTOCOL ===\n")
    for fn in [test_query_format, test_baseline_zero]:
        results.append((fn.__name__, fn(ser)))

    print("\n=== LEFT ENCODER ===\n")
    for fn in [
        test_left_encoder_forward,
        test_left_encoder_backward,
    ]:
        results.append((fn.__name__, fn(ser)))

    print("\n=== RIGHT ENCODER ===\n")
    for fn in [
        test_right_encoder_forward,
        test_right_encoder_backward,
    ]:
        results.append((fn.__name__, fn(ser)))

    print("\n=== INDEPENDENCE ===\n")
    for fn in [test_independence_left_only, test_independence_right_only]:
        results.append((fn.__name__, fn(ser)))

    print("\n=== RESOLUTION ===\n")
    for fn in [test_resolution_left, test_resolution_right]:
        results.append((fn.__name__, fn(ser)))

    print("\n=== ACCUMULATION ===\n")
    results.append((test_query_accumulates.__name__, test_query_accumulates(ser)))

    ser.close()

    print("\n" + "=" * 60)
    print("ENCODER TEST RESULTS")
    print("=" * 60)
    passed_n = sum(1 for _, p in results if p)
    failed_n = sum(1 for _, p in results if not p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n  {passed_n} passed  |  {failed_n} failed  |  {len(results)} total")

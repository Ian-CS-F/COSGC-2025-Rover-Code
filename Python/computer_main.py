"""
computer_main.py
Runs all rover navigation logic on a laptop/desktop instead of a Raspberry Pi.

Hardware setup:
    Motor Arduino → MOTOR_PORT  (runs Arduino/rover_interpreter.ino)
    IMU + LIDARs  → Raspberry Pi I2C bus 1 (GPIO 2/3) — read directly via smbus2
                    ICM-20948 @ 0x68, LIDAR-Lite v4 #1 @ 0x64, #2 @ 0x62

Set HARDWARE_MODE = False to run with no hardware at all — all sensors are
mocked so the navigation logic can be tested on any machine.

Controls (keyboard mode):
    W / ↑      Forward          A / ←   Turn left
    S / ↓      Reverse          D / →   Turn right
    + / =      Speed up         - / _   Speed down
    SPACE      Toggle autonomous / keyboard mode
    Q / ESC    Quit

Install dependencies:
    pip install pyserial pynput
"""

import math
import os
import sys
import time
import threading

# ── Configuration — edit these for your machine ───────────────────────────────
HARDWARE_MODE = True          # False = mock sensors, no hardware required
MOTOR_PORT    = "/dev/ttyACM0"   # port for the motor/encoder Arduino
BAUD_RATE     = 9600

# ── Path setup — finds nav modules relative to this file ─────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "Control", "Nav"))
sys.path.insert(0, os.path.join(_HERE, "Hardware"))

from heightmap import Heightmap   # type: ignore
from astar import AStar           # type: ignore

# ── Serial / motor constants (mirror of main.py) ──────────────────────────────
SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT,  RIGHT, UP,   DOWN         = 0, 1, 2, 3

HANDSHAKE_TIMEOUT   = 10
MOTOR_DONE_TIMEOUT  = 30.0
TURN_SPEED          = 0.5
TURN_TOLERANCE_DEG  = 5.0
TURN_TIMEOUT        = 15.0
SWEEP_TIMEOUT_S     = 300.0

DEFAULT_DRIVE_SPEED = 0.7
MIN_DRIVE_SPEED     = 0.3
SLIP_THRESHOLD      = 0.15

# ── Map / navigation constants (mirror of main.py) ────────────────────────────
MAP_WIDTH_M  = 3.0
MAP_HEIGHT_M = 8.0
RESOLUTION_M = 0.05
COLS = int(MAP_WIDTH_M  / RESOLUTION_M)
ROWS = int(MAP_HEIGHT_M / RESOLUTION_M)

GOAL_TOLERANCE_CELLS  = 3
SWEEP_EVERY_N_STEPS   = 5
STRAIGHT_SEGMENT_CM   = 50.0
STRAIGHT_MAX_SEGMENTS = 20
CLIFF_DROP_CM         = 5
PITCH_UP_THRESHOLD    = 30.0
FLIP_DETECT_THRESHOLD = 0.5

# ── Global rover state ────────────────────────────────────────────────────────
_flipped     = False
_drive_speed = DEFAULT_DRIVE_SPEED


# ═════════════════════════════════════════════════════════════════════════════
# SENSOR LAYER
# Either reads from sensor_bridge (HARDWARE_MODE=True) or returns mock values.
# ═════════════════════════════════════════════════════════════════════════════

if HARDWARE_MODE:
    import sensor_bridge  # type: ignore

    def _connect_sensors():
        print("Connecting to sensors via I2C...")
        sensor_bridge.connect()
        if not sensor_bridge.is_imu_ok():
            print("WARNING: IMU not responding — check wiring and i2c_bridge firmware")
        if not sensor_bridge.is_lidar_ok():
            print("WARNING: LIDAR not responding — check wiring")

    def _raw_imu() -> dict:
        """Return latest IMU dict, or a safe default if unavailable."""
        imu = sensor_bridge.read_imu()
        return imu if imu is not None else {"ax": 0.0, "ay": 0.0, "az": 1.0,
                                            "gx": 0.0, "gy": 0.0, "gz": 0.0,
                                            "mx": 100,  "my": 0,   "mz": 0}

    def read_lidar() -> float:
        dist = sensor_bridge.read_lidar()
        return dist if dist is not None else 2.0

else:
    # ── Mock sensor layer ─────────────────────────────────────────────────────
    # Returns static values that keep the rover upright and facing north.
    # Navigation logic runs without any hardware connected.
    def _connect_sensors():
        print("HARDWARE_MODE=False — using mocked sensor data")

    def _raw_imu() -> dict:
        return {"ax": 0.0, "ay": 0.0, "az": 1.0,
                "gx": 0.0, "gy": 0.0, "gz": 0.0,
                "mx": 100,  "my": 0,   "mz": 0}

    def read_lidar() -> float:
        return 2.0


# ── Computed sensor values (same math as main.py) ─────────────────────────────
def read_accel_z() -> float:
    return _raw_imu()["az"]


def read_heading() -> float:
    """Tilt-compensated compass heading in degrees (0–360, 0 = North)."""
    imu = _raw_imu()
    ax, ay, az = imu["ax"], imu["ay"], imu["az"]
    mx, my, mz = imu["mx"], imu["my"], imu["mz"]

    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    mx_c = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_c = (mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(-my_c, mx_c)) % 360
    return (heading + 180) % 360 if _flipped else heading


def read_pitch() -> float:
    """Rover pitch in degrees (positive = nose up)."""
    imu  = _raw_imu()
    ax, ay, az = imu["ax"], imu["ay"], imu["az"]
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    return -pitch if _flipped else pitch


# ═════════════════════════════════════════════════════════════════════════════
# SERIAL / MOTOR LAYER  (identical to main.py — no smbus2 involved)
# ═════════════════════════════════════════════════════════════════════════════

import serial as _serial   # avoid name clash with module


class CliffDetected(Exception):
    def __init__(self, partial_cm: float):
        self.partial_cm = partial_cm


def init_serial(port: str = MOTOR_PORT) -> "_serial.Serial":
    """Open the motor Arduino's port and complete the READY/ACK handshake."""
    ser = _serial.Serial(port, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            return ser
    raise RuntimeError(f"Motor Arduino did not send READY within {HANDSHAKE_TIMEOUT} s")


def send_command(ser, system: int, direction: int, amount: float, speed: float):
    packet = f"({system},{direction},{amount:.3f},{speed:.3f})\n"
    ser.write(packet.encode())


def drive_segment(ser, direction: int, distance_cm: float, speed: float) -> tuple[float, float]:
    """Send one distance-controlled motor command, wait for DONE or CLIFF."""
    send_command(ser, MOTOR, direction, distance_cm, speed)
    deadline = time.monotonic() + MOTOR_DONE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DONE:"):
            # DONE:<left_cm>,<right_cm>,<left_ratio>,<right_ratio>
            parts = line[5:].split(",")
            left_cm     = float(parts[0])
            right_cm    = float(parts[1]) if len(parts) > 1 else left_cm
            left_ratio  = float(parts[2]) if len(parts) > 2 else 1.0
            right_ratio = float(parts[3]) if len(parts) > 3 else 1.0
            return (left_cm + right_cm) / 2.0, (left_ratio + right_ratio) / 2.0
        if line.startswith("CLIFF:"):
            # CLIFF:<left_cm>,<right_cm>
            parts = line[6:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            raise CliffDetected((left_cm + right_cm) / 2.0)
    return 0.0, 0.0   # timeout


def move_rover(ser, direction: int, total_cm: float) -> float:
    """Drive total_cm with adaptive slip compensation. Returns distance covered."""
    global _drive_speed
    remaining = total_cm
    travelled = 0.0
    while remaining > 1.0:
        actual, error_ratio = drive_segment(ser, direction, remaining, _drive_speed)
        if actual == 0.0 and error_ratio == 0.0:
            break   # timed out
        travelled += actual
        remaining -= actual
        if abs(error_ratio - 1.0) > SLIP_THRESHOLD:
            _drive_speed = max(MIN_DRIVE_SPEED, _drive_speed / error_ratio)
    return travelled


def turn_to_heading(ser, target_deg: float):
    """Rotate in-place until IMU heading is within TURN_TOLERANCE_DEG of target."""
    current = read_heading()
    diff    = ((target_deg - current) + 180.0) % 360.0 - 180.0
    if abs(diff) < TURN_TOLERANCE_DEG:
        return
    direction = RIGHT if diff > 0.0 else LEFT
    send_command(ser, MOTOR, direction, 0.0, TURN_SPEED)
    deadline = time.monotonic() + TURN_TIMEOUT
    while time.monotonic() < deadline:
        current   = read_heading()
        remaining = ((target_deg - current) + 180.0) % 360.0 - 180.0
        if abs(remaining) < TURN_TOLERANCE_DEG:
            break
        if (diff > 0.0) != (remaining > 0.0):   # overshot — reverse
            diff = remaining
            direction = RIGHT if diff > 0.0 else LEFT
            send_command(ser, MOTOR, direction, 0.0, TURN_SPEED)
    send_command(ser, MOTOR, UP, 0.0, 0.0)   # stop


def query_distance(ser) -> float:
    """Ask the Arduino for encoder distance since the last query."""
    send_command(ser, QUERY, 0, 0.0, 0.0)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            # DIST:<left_cm>,<right_cm>
            parts = line[5:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            return (left_cm + right_cm) / 2.0
    return 0.0


def check_and_update_flip(ser):
    """Detect rover inversion from IMU and notify the Arduino on state changes."""
    global _flipped
    az = read_accel_z()
    now_flipped = az < -FLIP_DETECT_THRESHOLD
    if now_flipped != _flipped:
        _flipped = now_flipped
        send_command(ser, FLIP, 0, 1.0 if _flipped else 0.0, 0.0)


def update_position(x, y, distance_cm, heading_deg, initial_heading_deg) -> tuple[int, int]:
    dist_m       = distance_cm / 100.0
    relative_rad = math.radians(heading_deg - initial_heading_deg)
    d_row = round(dist_m * math.cos(relative_rad) / RESOLUTION_M)
    d_col = round(dist_m * math.sin(relative_rad) / RESOLUTION_M)
    return x + d_row, y + d_col


# ═════════════════════════════════════════════════════════════════════════════
# NAVIGATION  (mirrors main.py — sensor calls go through this file's layer)
# ═════════════════════════════════════════════════════════════════════════════

def sensor_sweep(ser, heightmap: Heightmap, x: int, y: int,
                 range_deg: float = 90.0, step_ms: float = 200.0):
    rover_pitch = read_pitch()
    send_command(ser, SWEEP, 0, range_deg, step_ms)
    deadline = time.monotonic() + SWEEP_TIMEOUT_S
    while True:
        if time.monotonic() > deadline:
            print("WARNING: sweep timed out")
            break
        line = ser.readline().decode(errors="replace").strip()
        if line == "SWEEP_DONE":
            break
        if line.startswith("AT:"):
            parts = line[3:].split(",")
            if len(parts) != 2:
                continue
            h_angle        = float(parts[0])
            t_angle        = float(parts[1])
            effective_tilt = t_angle + rover_pitch
            distance       = read_lidar()
            h_rad = math.radians(h_angle)
            t_rad = math.radians(effective_tilt)
            d_horiz   = distance * math.cos(t_rad)
            dz        = distance * math.sin(t_rad)
            d_forward = d_horiz * math.cos(h_rad)
            d_side    = d_horiz * math.sin(h_rad)
            tr = x + round(d_forward / RESOLUTION_M)
            tc = y + round(d_side    / RESOLUTION_M)
            if 0 <= tr < heightmap.rows and 0 <= tc < heightmap.cols:
                heightmap.heights[tr][tc] = round(dz * 100)
            ser.write(b"NEXT\n")


def handle_cliff(ser, heightmap: Heightmap, x: int, y: int,
                 heading_deg: float, initial_heading_deg: float):
    if read_pitch() > PITCH_UP_THRESHOLD:
        return
    rel_rad   = math.radians(heading_deg - initial_heading_deg)
    ahead_row = x + round(math.cos(rel_rad))
    ahead_col = y + round(math.sin(rel_rad))
    if 0 <= ahead_row < heightmap.rows and 0 <= ahead_col < heightmap.cols:
        heightmap.heights[ahead_row][ahead_col] = heightmap.heights[x][y] - CLIFF_DROP_CM
    sensor_sweep(ser, heightmap, x, y, range_deg=360.0, step_ms=200.0)
    heightmap.compute_slopes()


def follow_path(ser, heightmap: Heightmap, planner: AStar,
                path: list, x: int, y: int, initial_heading: float,
                goal: tuple) -> tuple[int, int]:
    steps = 0
    while len(path) > 1:
        check_and_update_flip(ser)
        if steps >= SWEEP_EVERY_N_STEPS:
            sensor_sweep(ser, heightmap, x, y)
            heightmap.compute_slopes()
            path  = planner.find_path((x, y), goal) or []
            steps = 0
            if not path:
                print("No path after periodic sweep — stopping")
                break
        next_cell      = path[1]
        dr, dc         = next_cell[0] - x, next_cell[1] - y
        rel_deg        = math.degrees(math.atan2(dc, dr))
        target_heading = (initial_heading + rel_deg) % 360.0
        dist_cm        = math.hypot(dr, dc) * RESOLUTION_M * 100.0
        turn_to_heading(ser, target_heading)
        heading = read_heading()
        try:
            moved = move_rover(ser, UP, dist_cm)
            x, y  = update_position(x, y, moved, heading, initial_heading)
            path  = path[1:]
            steps += 1
        except CliffDetected as e:
            x, y = update_position(x, y, e.partial_cm, heading, initial_heading)
            handle_cliff(ser, heightmap, x, y, read_heading(), initial_heading)
            path  = planner.find_path((x, y), goal) or []
            steps = 0
            if not path:
                print("No path after cliff — stopping")
                break
    return x, y


# ═════════════════════════════════════════════════════════════════════════════
# KEYBOARD MODE
# ═════════════════════════════════════════════════════════════════════════════

try:
    from pynput import keyboard as _kb
    _PYNPUT = True
except ImportError:
    _PYNPUT = False


def keyboard_mode(ser):
    """
    Drive manually with WASD / arrow keys. Press SPACE to hand off to the
    autonomous navigator for one full path attempt, then return here.
    Press Q or ESC to quit.
    """
    if not _PYNPUT:
        print("pynput is not installed.  Run:  pip install pynput")
        return

    _keys   = set()
    _speed  = [0.5]
    _mode   = ["keyboard"]    # list so closures can mutate it
    _quit   = threading.Event()
    _space_edge = [False]     # rising-edge detector for spacebar

    def _press(key):
        try:
            _keys.add(key.char.lower())
        except AttributeError:
            _keys.add(key)

    def _release(key):
        try:
            _keys.discard(key.char.lower())
        except AttributeError:
            _keys.discard(key)

    listener = _kb.Listener(on_press=_press, on_release=_release)
    listener.start()

    last_motor_cmd = None    # track last command sent to avoid serial spam

    def _drive_cmd():
        """Return the current drive direction from held keys, or None."""
        fwd = "w" in _keys or _kb.Key.up    in _keys
        rev = "s" in _keys or _kb.Key.down  in _keys
        lft = "a" in _keys or _kb.Key.left  in _keys
        rgt = "d" in _keys or _kb.Key.right in _keys
        if fwd: return UP
        if rev: return DOWN
        if lft: return LEFT
        if rgt: return RIGHT
        return None

    # Print initial blank display area so subsequent reprints overwrite cleanly
    DISPLAY_LINES = 7
    print("\n" * DISPLAY_LINES, end="")

    try:
        while not _quit.is_set():
            # ── Quit ─────────────────────────────────────────────────────────
            if "q" in _keys or _kb.Key.esc in _keys:
                break

            # ── Speed adjustment (one step per key-down event) ────────────────
            if "+" in _keys or "=" in _keys:
                _speed[0] = min(1.0, round(_speed[0] + 0.05, 2))
                time.sleep(0.15)   # simple debounce
            if "-" in _keys or "_" in _keys:
                _speed[0] = max(0.1, round(_speed[0] - 0.05, 2))
                time.sleep(0.15)

            # ── Mode toggle (spacebar rising edge) ────────────────────────────
            space_held = _kb.Key.space in _keys or " " in _keys
            if space_held and not _space_edge[0]:
                _space_edge[0] = True
                if _mode[0] == "keyboard":
                    _mode[0] = "autonomous"
                    send_command(ser, MOTOR, UP, 0.0, 0.0)   # stop before handing off
                    last_motor_cmd = None
                else:
                    _mode[0] = "keyboard"
            elif not space_held:
                _space_edge[0] = False

            # ── Motor output ──────────────────────────────────────────────────
            if _mode[0] == "keyboard":
                cmd = _drive_cmd()
                if cmd != last_motor_cmd:
                    if cmd is None:
                        send_command(ser, MOTOR, UP, 0.0, 0.0)     # stop
                    else:
                        send_command(ser, MOTOR, cmd, 0.0, _speed[0])  # continuous
                    last_motor_cmd = cmd

            # ── Read sensors for display ──────────────────────────────────────
            heading = read_heading()
            pitch   = read_pitch()
            lidar   = read_lidar()
            # Non-blocking encoder query — skip if serial is busy with a drive
            try:
                encoder = query_distance(ser) if _mode[0] == "keyboard" else 0.0
            except Exception:
                encoder = 0.0

            dir_label = {UP: "FORWARD", DOWN: "REVERSE",
                         LEFT: "LEFT",  RIGHT: "RIGHT"}.get(last_motor_cmd, "STOPPED")
            hw_label  = "REAL" if HARDWARE_MODE else "MOCK"

            # Move cursor up to overwrite previous display
            print(f"\033[{DISPLAY_LINES}A", end="")
            print(f"=== COSGC Rover  |  {_mode[0].upper()} mode  |  HW: {hw_label} ===          ")
            print(f"  Heading : {heading:6.1f} deg     Pitch  : {pitch:5.1f} deg          ")
            print(f"  LiDAR   : {lidar:6.2f} m       Encoder: {encoder:6.1f} cm           ")
            print(f"  Speed   : {_speed[0]:.2f}             Driving: {dir_label:<10}          ")
            print(f"                                                            ")
            print(f"  W/S/A/D or arrows = drive    +/- = speed                 ")
            print(f"  SPACE = toggle auto/keyboard  Q/ESC = quit                ")

            time.sleep(0.1)

    finally:
        send_command(ser, MOTOR, UP, 0.0, 0.0)   # always stop on exit
        listener.stop()
        print("\nKeyboard mode exited.")


# ═════════════════════════════════════════════════════════════════════════════
# AUTONOMOUS MODE
# ═════════════════════════════════════════════════════════════════════════════

def autonomous_mode(ser):
    """Full A*-guided navigation from start to goal."""
    initial_heading = read_heading()
    print(f"Initial heading: {initial_heading:.1f} deg")

    heightmap = Heightmap(ROWS, COLS)
    planner   = AStar(heightmap, diagonal=True, turn_penalty=1.0, max_height_diff=2)
    x, y      = ROWS - 1, COLS // 2
    goal      = (0, COLS // 2)

    print(f"Map: {ROWS} rows x {COLS} cols  ({RESOLUTION_M} m/cell)")
    print("Initial sweep...")
    sensor_sweep(ser, heightmap, x, y, range_deg=90.0, step_ms=200.0)
    heightmap.compute_slopes()

    path = planner.find_path((x, y), goal)
    if path is None:
        print("No initial path found — driving straight...")
        for _ in range(STRAIGHT_MAX_SEGMENTS):
            check_and_update_flip(ser)
            heading = read_heading()
            try:
                moved = move_rover(ser, UP, STRAIGHT_SEGMENT_CM)
            except CliffDetected as e:
                moved = e.partial_cm
            x, y = update_position(x, y, moved, heading, initial_heading)
            sensor_sweep(ser, heightmap, x, y, range_deg=90.0, step_ms=200.0)
            heightmap.compute_slopes()
            path = planner.find_path((x, y), goal)
            if path:
                break
        if path is None:
            print("Could not find a path — stopped")
            return

    print(f"Path found: {len(path)} cells   Goal: {goal}")

    while True:
        check_and_update_flip(ser)
        x, y = follow_path(ser, heightmap, planner, path, x, y, initial_heading, goal)

        if math.hypot(x - goal[0], y - goal[1]) <= GOAL_TOLERANCE_CELLS:
            print(f"Goal reached at ({x}, {y})")
            break

        print("Replanning...")
        sensor_sweep(ser, heightmap, x, y, range_deg=360.0, step_ms=200.0)
        heightmap.compute_slopes()
        path = planner.find_path((x, y), goal)
        if path is None:
            print("No path after full sweep — stopped")
            break


# ═════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("COSGC Rover — Computer Development Mode")
    print(f"  Hardware : {'REAL' if HARDWARE_MODE else 'MOCK (no hardware needed)'}")
    print(f"  Motor    : {MOTOR_PORT}")
    if HARDWARE_MODE:
        print(f"  Sensor   : {SENSOR_PORT}")
    print()

    # Connect sensors first (may be a no-op in mock mode)
    _connect_sensors()

    # Connect to the motor Arduino (skip in full-mock mode if no hardware)
    ser = None
    if HARDWARE_MODE:
        print(f"Connecting to motor Arduino on {MOTOR_PORT}...")
        ser = init_serial(MOTOR_PORT)
        print("Motor Arduino connected.")
    else:
        # In mock mode we still need a serial object for keyboard mode to work.
        # If you have a motor Arduino but mocked sensors, set HARDWARE_MODE=True
        # and only disconnect the sensor Arduino.
        print("Mock mode — motor commands will be printed, not sent.")

        class _MockSerial:
            """Fake serial port that prints commands instead of sending them."""
            def write(self, data):
                print(f"  [MOTOR CMD] {data.decode().strip()}")
            def readline(self):
                time.sleep(0.05)
                return b"DONE:10.0,10.0,1.0,1.0\n"
            @property
            def in_waiting(self):
                return 0

        ser = _MockSerial()

    print()
    choice = input("Mode? [k]eyboard (default) / [a]utonomous: ").strip().lower()
    try:
        if choice.startswith("a"):
            autonomous_mode(ser)
        else:
            keyboard_mode(ser)
    finally:
        if HARDWARE_MODE and ser:
            ser.close()
        if HARDWARE_MODE:
            sensor_bridge.disconnect()

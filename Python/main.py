"""
Communication Schema:
(System, Direction, Amount, Speed)

System:
    Servo (0), Motor (1), Sweep (2)

Direction:
    left (0), right (1), up (2), down (3)
    — ignored for Sweep

Amount:
    Servo: target angle as 0.0–1.0 (→ 0–180°)
    Motor: target distance in cm (0 = continuous, no distance control)
    Sweep: total horizontal range in degrees (e.g. 90.0)

Speed:
    Motor: PWM duty cycle 0.0–1.0 (→ 0–255)
    Sweep: time between tilt steps in ms (e.g. 200.0)

Motor distance protocol:
    Pi sends      (1, <dir>, <distance_cm>, <speed>)
    Arduino drives until encoder reaches target, then stops
    Arduino sends "DONE:<actual_cm>,<error_ratio>"
      actual_cm   — encoder-measured distance covered
      error_ratio — actual_counts / target_counts
                    > 1.0 : wheels spun more than expected (slipping)
                    < 1.0 : fewer counts than expected (stalled / very slow)
    (amount == 0 → continuous drive; no DONE sent)

Sweep protocol:
    Pi sends      (2, 0, <range_deg>, <step_ms>)
    Arduino sends "AT:<h_angle>,<tilt_angle>"  at each position
    Pi sends      "NEXT"                        when ready for the next step
    Arduino sends "SWEEP_DONE"                  when back at centre

Arduino → Pi:
    READY              sent on boot (every second until ACK received)
    DONE:<cm>,<ratio>  drive segment complete; encoder distance + slip ratio
    CLIFF:<cm>         ground lost mid-drive (motors already stopped);
                       cm = partial encoder distance covered before stop.
                       No DONE is sent for that segment.
                       Also sent at 1 Hz while idle if ground is lost.
    AT:h,t             current horizontal and tilt angles during a sweep
    SWEEP_DONE         sweep complete, rover back at heading 0
    DIST:<cm>          response to Query command (distance since last query)
"""

import math
import struct
import sys
import time
import serial
import smbus2  # type: ignore

sys.path.append("Control/Nav")
from heightmap import Heightmap  # type: ignore
from astar import AStar          # type: ignore


class CliffDetected(Exception):
    """Raised when the Arduino reports a cliff mid-drive."""
    def __init__(self, partial_cm: float) -> None:
        self.partial_cm = partial_cm  # encoder distance covered before stop

# ── Serial ────────────────────────────────────────────────────────────────────
SERIAL_PORT       = "/dev/ttyUSB0"
BAUD_RATE         = 9600
HANDSHAKE_TIMEOUT = 10  # seconds to wait for READY

# ── Command constants ─────────────────────────────────────────────────────────
SERVO, MOTOR, SWEEP, QUERY, FLIP = 0, 1, 2, 3, 4
LEFT, RIGHT, UP, DOWN = 0, 1, 2, 3

# ── LIDAR (Garmin LIDAR-Lite v4 LED) ─────────────────────────────────────────
LIDAR_ADDR = 0x62
I2C_BUS    = 1

# ── IMU (ICM-20948) ───────────────────────────────────────────────────────────
ICM_ADDR     = 0x68   # use 0x69 if AD0 is pulled high
REG_BANK_SEL = 0x7F   # write (bank << 4) to select register bank

# Bank 0
_PWR_MGMT_1   = 0x06
_INT_PIN_CFG  = 0x0F
_ACCEL_XOUT_H = 0x2D

# AK09916 magnetometer (exposed on main I2C bus via bypass mode)
MAG_ADDR  = 0x0C
_MAG_CNTL2 = 0x31  # control 2 — measurement mode
_MAG_ST1   = 0x10  # status 1  — DRDY bit 0
_MAG_HXL   = 0x11  # first measurement byte (little-endian X/Y/Z + ST2)

# ── Flip detection ────────────────────────────────────────────────────────────
FLIP_DETECT_THRESHOLD = 0.5   # |az| below −threshold → rover is inverted

_flipped: bool = False  # tracks current flip state; updated by check_and_update_flip

# ── Cliff detection ───────────────────────────────────────────────────────────
CLIFF_DROP_CM      = 5     # cliff cell is marked this many cm below current cell
PITCH_UP_THRESHOLD = 30.0  # degrees — above this, cliff alert is a normal slope

# ── Map configuration ─────────────────────────────────────────────────────────
MAP_WIDTH_M  = 3.0   # physical width  in metres
MAP_HEIGHT_M = 8.0   # physical height in metres
RESOLUTION_M = 0.05  # metres per cell

COLS = int(MAP_WIDTH_M  / RESOLUTION_M)
ROWS = int(MAP_HEIGHT_M / RESOLUTION_M)


# ── Serial init ───────────────────────────────────────────────────────────────
def init_serial() -> serial.Serial:
    """Open the serial port and complete the READY/ACK handshake."""
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()

    deadline = time.monotonic() + HANDSHAKE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            print("Arduino handshake complete")
            return ser

    raise RuntimeError(f"Arduino did not send READY within {HANDSHAKE_TIMEOUT} s")


def send_command(ser: serial.Serial, system: int, direction: int, amount: float, speed: float) -> None:
    """Send a (System, Direction, Amount, Speed) command to the Arduino."""
    packet = f"({system},{direction},{amount:.3f},{speed:.3f})\n"
    ser.write(packet.encode())


# ── IMU functions ────────────────────────────────────────────────────────────
def _icm_bank(bus: smbus2.SMBus, bank: int) -> None:
    bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, bank << 4)


def read_accel_z(bus: smbus2.SMBus) -> float:
    """Return the Z-axis accelerometer reading in g (≈ +1 normal, ≈ −1 inverted)."""
    _icm_bank(bus, 0)
    raw = bus.read_i2c_block_data(ICM_ADDR, _ACCEL_XOUT_H, 6)
    return struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0


def check_and_update_flip(ser: serial.Serial, bus: smbus2.SMBus) -> None:
    """
    Detect rover inversion from the IMU Z-axis and inform the Arduino if the
    flip state changes.  Sends FLIP only on transitions to avoid serial spam.
    """
    global _flipped
    az = read_accel_z(bus)
    now_flipped = az < -FLIP_DETECT_THRESHOLD
    if now_flipped != _flipped:
        _flipped = now_flipped
        send_command(ser, FLIP, 0, 1.0 if _flipped else 0.0, 0.0)


def init_imu(bus: smbus2.SMBus) -> None:
    """Wake the ICM-20948 and enable I2C bypass so the AK09916 is accessible."""
    _icm_bank(bus, 0)
    bus.write_byte_data(ICM_ADDR, _PWR_MGMT_1,  0x01)  # wake + auto-clock
    time.sleep(0.1)
    bus.write_byte_data(ICM_ADDR, _INT_PIN_CFG, 0x02)  # BYPASS_EN
    time.sleep(0.02)
    bus.write_byte_data(MAG_ADDR, _MAG_CNTL2,   0x08)  # AK09916 continuous 100 Hz
    time.sleep(0.02)


def read_heading(bus: smbus2.SMBus) -> float:
    """
    Return the tilt-compensated compass heading in degrees (0–360, 0 = North).
    Requires init_imu() to have been called first.
    """
    # ── Accelerometer — used for tilt compensation ────────────────────────────
    _icm_bank(bus, 0)
    raw = bus.read_i2c_block_data(ICM_ADDR, _ACCEL_XOUT_H, 6)
    ax = struct.unpack(">h", bytes(raw[0:2]))[0] / 16384.0  # ±2 g scale
    ay = struct.unpack(">h", bytes(raw[2:4]))[0] / 16384.0
    az = struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0

    # ── Magnetometer ──────────────────────────────────────────────────────────
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        if bus.read_byte_data(MAG_ADDR, _MAG_ST1) & 0x01:  # DRDY
            break
    raw = bus.read_i2c_block_data(MAG_ADDR, _MAG_HXL, 8)   # X/Y/Z + ST2
    mx = struct.unpack("<h", bytes(raw[0:2]))[0]  # AK09916 is little-endian
    my = struct.unpack("<h", bytes(raw[2:4]))[0]
    mz = struct.unpack("<h", bytes(raw[4:6]))[0]
    # Reading ST2 (raw[7]) releases the data latch automatically

    # ── Tilt-compensated heading ──────────────────────────────────────────────
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    mx_c = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_c = (mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(-my_c, mx_c)) % 360
    # When inverted, effective forward direction is 180° opposite the compass
    return (heading + 180) % 360 if _flipped else heading


def read_pitch(bus: smbus2.SMBus) -> float:
    """
    Return the rover's pitch angle in degrees (positive = nose up).
    Requires init_imu() to have been called first.
    """
    _icm_bank(bus, 0)
    raw = bus.read_i2c_block_data(ICM_ADDR, _ACCEL_XOUT_H, 6)
    ax = struct.unpack(">h", bytes(raw[0:2]))[0] / 16384.0
    ay = struct.unpack(">h", bytes(raw[2:4]))[0] / 16384.0
    az = struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    # When inverted, the pitch sign convention is reversed relative to forward travel
    return -pitch if _flipped else pitch


# ── LIDAR ─────────────────────────────────────────────────────────────────────
def read_lidar(bus: smbus2.SMBus) -> float:
    """Trigger a measurement and return distance in metres."""
    bus.write_byte_data(LIDAR_ADDR, 0x00, 0x04)  # trigger
    time.sleep(0.02)                               # ~20 ms measurement time
    high = bus.read_byte_data(LIDAR_ADDR, 0x0f)
    low  = bus.read_byte_data(LIDAR_ADDR, 0x10)
    return ((high << 8) | low) / 100.0            # cm → m


# ── Motor PID constants ───────────────────────────────────────────────────────
MOTOR_DONE_TIMEOUT  = 30.0  # seconds to wait for DONE per segment
SLIP_THRESHOLD      = 0.15  # error deviation that triggers speed update
MIN_DRIVE_SPEED     = 0.3   # lowest PWM fraction allowed (avoid motor stall)
DEFAULT_DRIVE_SPEED = 0.7   # starting speed before any learning

_drive_speed: float = DEFAULT_DRIVE_SPEED  # learned optimal — persists across calls


def drive_segment(
    ser: serial.Serial, direction: int, distance_cm: float, speed: float
) -> tuple[float, float]:
    """
    Send one distance-controlled motor command and wait for DONE.

    Returns (actual_cm, error_ratio) where:
        actual_cm   — encoder-measured distance covered
        error_ratio — actual_counts / target_counts
                      > 1.0 : slipping (wheels spun more than expected)
                      < 1.0 : stalled or very slow
    """
    send_command(ser, MOTOR, direction, distance_cm, speed)
    deadline = time.monotonic() + MOTOR_DONE_TIMEOUT
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DONE:"):
            parts = line[5:].split(",")
            actual_cm   = float(parts[0])
            error_ratio = float(parts[1]) if len(parts) > 1 else 1.0
            return actual_cm, error_ratio
        if line.startswith("CLIFF:"):
            raise CliffDetected(float(line[6:]))
    return 0.0, 0.0


def move_rover(ser: serial.Serial, direction: int, total_cm: float) -> float:
    """
    Drive total_cm in direction with slip-compensating closed-loop control.

    Uses and updates the module-level _drive_speed so the learned optimal
    speed carries over into every subsequent call.

    Returns the total encoder-measured distance covered (cm).
    """
    global _drive_speed
    remaining = total_cm
    travelled = 0.0

    while remaining > 1.0:
        segment = remaining
        actual, error_ratio = drive_segment(ser, direction, segment, _drive_speed)
        travelled += actual
        remaining -= actual

        # Update the persistent speed whenever error deviates past threshold.
        # error_ratio > 1 → slipping (reduce speed for better grip)
        # error_ratio < 1 → stalling (reduce speed to avoid motor strain)
        if abs(error_ratio - 1.0) > SLIP_THRESHOLD:
            _drive_speed = max(MIN_DRIVE_SPEED, _drive_speed / error_ratio)

    return travelled


# ── Odometry ──────────────────────────────────────────────────────────────────
def query_distance(ser: serial.Serial) -> float:
    """
    Ask the Arduino for distance travelled since the last query.
    Returns centimetres (positive = forward, negative = backward).
    """
    send_command(ser, QUERY, 0, 0.0, 0.0)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DIST:"):
            return float(line[5:])
    return 0.0


def update_position(
    x: int,
    y: int,
    distance_cm: float,
    heading_deg: float,
    initial_heading_deg: float,
) -> tuple[int, int]:
    """
    Dead-reckon a new grid position from distance and compass heading.

    Row increases in the direction of initial_heading (forward).
    Col increases 90° clockwise from initial_heading (right).

    Args:
        x, y:                current grid position (row, col)
        distance_cm:         distance travelled since last update (cm)
        heading_deg:         current compass heading (0–360, 0 = North)
        initial_heading_deg: heading at startup — defines the forward axis
    """
    distance_m = distance_cm / 100.0
    relative_rad = math.radians(heading_deg - initial_heading_deg)

    d_row = round(distance_m * math.cos(relative_rad) / RESOLUTION_M)
    d_col = round(distance_m * math.sin(relative_rad) / RESOLUTION_M)
    return x + d_row, y + d_col


# ── Sensor sweep ──────────────────────────────────────────────────────────────
def sensor_sweep(
    ser: serial.Serial,
    bus: smbus2.SMBus,
    heightmap: Heightmap,
    x: int,
    y: int,
    range_deg: float = 90.0,
    step_ms:   float = 200.0,
) -> None:
    """
    Command the Arduino to perform a sweep, then use each LIDAR reading
    to compute a heightmap cell and update its height.

    The rover's current pitch is read from the IMU before the sweep begins
    and subtracted from every reported tilt angle so that height calculations
    are always relative to true horizontal, not the rover's body frame.

    Args:
        bus:       shared SMBus instance (used for LIDAR + IMU)
        x, y:      rover's current grid position (row, col)
        range_deg: total horizontal sweep range in degrees
        step_ms:   wait time per tilt step in ms
    """
    rover_pitch = read_pitch(bus)  # degrees, positive = nose up
    send_command(ser, SWEEP, 0, range_deg, step_ms)

    while True:
        line = ser.readline().decode(errors="replace").strip()

        if line == "SWEEP_DONE":
            break

        if line.startswith("AT:"):
            parts = line[3:].split(",")
            if len(parts) != 2:
                continue

            h_angle = float(parts[0])   # horizontal (deg, 0 = forward)
            t_angle = float(parts[1])   # tilt reported by Arduino (deg)

            # Add rover pitch so the effective angle is relative to ground
            effective_tilt = t_angle + rover_pitch

            distance = read_lidar(bus)  # metres

            h_rad = math.radians(h_angle)
            t_rad = math.radians(effective_tilt)

            d_horiz   = distance * math.cos(t_rad)  # ground-plane distance
            dz        = distance * math.sin(t_rad)  # height offset from sensor

            d_forward = d_horiz * math.cos(h_rad)   # metres ahead of rover
            d_side    = d_horiz * math.sin(h_rad)   # metres right of rover

            target_row = x + round(d_forward / RESOLUTION_M)
            target_col = y + round(d_side    / RESOLUTION_M)

            if 0 <= target_row < heightmap.rows and 0 <= target_col < heightmap.cols:
                heightmap.heights[target_row][target_col] = round(dz * 100)  # cm

            ser.write(b"NEXT\n")


# ── Cliff response ────────────────────────────────────────────────────────────
def handle_cliff(
    ser: serial.Serial,
    bus: smbus2.SMBus,
    heightmap: Heightmap,
    planner: AStar,
    x: int,
    y: int,
    heading_deg: float,
    initial_heading_deg: float,
) -> list[tuple[int, int]] | None:
    """
    Called when a CliffDetected exception propagates up from move_rover.

    1. Check IMU pitch — if the rover is pitched upward the ultrasonic is
       simply looking past the slope, not a real drop; return None to retry.
    2. Mark the cell directly ahead of the rover as a cliff in the heightmap
       (height = CLIFF_MARK_HEIGHT, well below current ground so A* avoids it).
    3. Perform a full 360° LIDAR sweep to refresh the map around the rover.
    4. Return a fresh path from the planner, or None if no path exists.
    """
    pitch = read_pitch(bus)
    if pitch > PITCH_UP_THRESHOLD:
        return None  # rover is climbing — expected ground loss, not a cliff

    # Mark cell immediately ahead as a cliff
    rel_rad    = math.radians(heading_deg - initial_heading_deg)
    ahead_row  = x + round(math.cos(rel_rad))
    ahead_col  = y + round(math.sin(rel_rad))
    if 0 <= ahead_row < heightmap.rows and 0 <= ahead_col < heightmap.cols:
        heightmap.heights[ahead_row][ahead_col] = heightmap.heights[x][y] - CLIFF_DROP_CM

    # Full 360° sweep to update surroundings before replanning
    sensor_sweep(ser, bus, heightmap, x, y, range_deg=360.0, step_ms=200.0)

    return None  # caller should call planner.find_path() with the updated map


# ── Boot ──────────────────────────────────────────────────────────────────────
def main() -> None:
    ser = init_serial()

    bus = smbus2.SMBus(I2C_BUS)
    init_imu(bus)
    initial_heading = read_heading(bus)
    print(f"Initial heading: {initial_heading:.1f}°")

    heightmap = Heightmap(ROWS, COLS)
    planner   = AStar(heightmap, diagonal=True, turn_penalty=1.0, max_height_diff=2)

    print(f"Map: {ROWS} rows × {COLS} cols  ({RESOLUTION_M} m/cell)")

    # Starting grid position — middle bottom (rover travels forward/up the grid)
    x, y = ROWS - 1, COLS // 2

    while True:
        # Detect flip; sends FLIP command to Arduino only on state change
        check_and_update_flip(ser, bus)

        # Update position from encoder + IMU
        dist_cm  = query_distance(ser)
        heading  = read_heading(bus)   # already corrected for flip state
        x, y     = update_position(x, y, dist_cm, heading, initial_heading)

        # TODO: sensor_sweep(ser, bus, heightmap, x, y, range_deg=90.0, step_ms=200.0)
        # TODO: determine goal
        # TODO: path = planner.find_path((x, y), goal)

        # TODO: replace with real path step iteration
        # Example movement with cliff handling:
        # try:
        #     moved = move_rover(ser, UP, step_cm)
        #     x, y = update_position(x, y, moved, heading, initial_heading)
        # except CliffDetected as e:
        #     x, y = update_position(x, y, e.partial_cm, heading, initial_heading)
        #     heading = read_heading(bus)
        #     handle_cliff(ser, bus, heightmap, planner, x, y, heading, initial_heading)
        #     path = planner.find_path((x, y), goal)  # replan with updated map
        pass


if __name__ == "__main__":
    main()

"""
Test Protocol: Height Map
=========================
Initialises both LIDAR-Lite v4 sensors (0x5b and 0x62), performs a simulated
vertical scan at a range of tilt angles, and builds a small Heightmap from the
readings.  Two grids are then printed in the terminal:

  Grid 1 — Elevation map
      Block density represents relative height above the lowest scanned cell.
          ░░  flat / very low  (bottom 25 % of height range)
          ▒▒  low-medium       (25 – 50 %)
          ▓▓  medium-high      (50 – 75 %)
          ██  steep / very high (top 25 %)
          ??  no data recorded for this cell

  Grid 2 — Traversability map
      Shows which cells A* would consider passable.
          ··  safe — max height diff to any neighbour ≤ MAX_HEIGHT_DIFF
          ██  obstacle — at least one neighbour differs by > MAX_HEIGHT_DIFF

Hardware required:
    LIDAR-Lite v4 #1 at I2C address 0x5b
    LIDAR-Lite v4 #2 at I2C address 0x62
    ICM-20948 IMU at 0x69 (optional — used for pitch compensation)

Run on the Pi:
    python3 "test_heightmap.py"

Use DEMO_MODE = True to run without any hardware.  Synthetic terrain is
generated so the visualisation and A* passability logic can be verified
on any machine.
"""

import math
import os
import struct
import sys
import time

# ── Hardware flag ─────────────────────────────────────────────────────────────
DEMO_MODE = False   # True = generate synthetic terrain, no hardware required

# ── I2C / sensor addresses ───────────────────────────────────────────────────
I2C_BUS      = 1
LIDAR_ADDR_1 = 0x5b   # LIDAR-Lite v4 LED #1
LIDAR_ADDR_2 = 0x62   # LIDAR-Lite v4 LED #2
ICM_ADDR     = 0x69   # ICM-20948 IMU
MAG_ADDR     = 0x0C   # AK09916 magnetometer (via I2C bypass)
REG_BANK_SEL = 0x7F
PWR_MGMT_1   = 0x06
INT_PIN_CFG  = 0x0F
ACCEL_XOUT_H = 0x2D

# ── Scan parameters ───────────────────────────────────────────────────────────
# Vertical tilt angles to sample (degrees).
# Negative = looking down, positive = looking up.
# Adjust to match your LIDAR mount angle range.
SCAN_ANGLES_DEG = list(range(-30, 31, 5))   # -30° … +30° in 5° steps

# Height at which the LIDAR is mounted above the ground (cm).
# Used to convert tilt-angle readings into absolute terrain height.
LIDAR_HEIGHT_CM = 20.0

# ── Map parameters ───────────────────────────────────────────────────────────
# Each cell is CELL_SIZE_CM × CELL_SIZE_CM on the ground.
CELL_SIZE_CM     = 10.0   # 10 cm per cell
MAP_FORWARD_ROWS = 20     # number of cells to model ahead of the rover
MAP_SIDE_COLS    = 10     # number of cells to model to each side (total = 2× + 1)

# A* passability threshold (mirrors main.py's max_height_diff).
# Cells whose height differs from a neighbour by more than this are obstacles.
MAX_HEIGHT_DIFF = 3   # cm

# ── Nav module path ───────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "Python", "Control", "Nav"))

from heightmap import Heightmap   # type: ignore
from astar import AStar           # type: ignore


# ═════════════════════════════════════════════════════════════════════════════
# HARDWARE LAYER
# ═════════════════════════════════════════════════════════════════════════════

def _read_one_lidar(bus, addr: int) -> float | None:
    """Trigger a LIDAR-Lite v4 measurement. Returns metres or None on error."""
    try:
        bus.write_byte_data(addr, 0x00, 0x04)   # initiate acquisition
        time.sleep(0.02)
        high = bus.read_byte_data(addr, 0x0f)
        low  = bus.read_byte_data(addr, 0x10)
        return ((high << 8) | low) / 100.0      # raw unit is cm → convert to m
    except OSError:
        return None


def _read_lidars(bus) -> float:
    """
    Read both LIDARs and return the best distance estimate in metres.
    Falls back to the closer sensor if they diverge by more than 10 cm,
    or to 2.0 m if both fail.
    """
    d1 = _read_one_lidar(bus, LIDAR_ADDR_1)
    d2 = _read_one_lidar(bus, LIDAR_ADDR_2)
    if d1 is None and d2 is None:
        return 2.0
    if d1 is None:
        return d2
    if d2 is None:
        return d1
    if abs(d1 - d2) <= 0.10:          # within 10 cm → average
        return (d1 + d2) / 2.0
    return min(d1, d2)                 # diverged → take closer reading


def _init_imu(bus) -> bool:
    """Wake the ICM-20948 and enable I2C bypass. Returns True on success."""
    try:
        bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, 0 << 4)
        bus.write_byte_data(ICM_ADDR, PWR_MGMT_1,   0x01)   # wake, auto clock
        time.sleep(0.1)
        bus.write_byte_data(ICM_ADDR, INT_PIN_CFG,  0x02)   # I2C bypass for mag
        time.sleep(0.02)
        return True
    except OSError:
        return False


def _read_pitch(bus) -> float:
    """Return rover pitch in degrees (positive = nose up), or 0.0 on error."""
    try:
        bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, 0 << 4)
        raw = bus.read_i2c_block_data(ICM_ADDR, ACCEL_XOUT_H, 6)
        ax = struct.unpack(">h", bytes(raw[0:2]))[0] / 16384.0
        ay = struct.unpack(">h", bytes(raw[2:4]))[0] / 16384.0
        az = struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0
        return math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    except OSError:
        return 0.0


# ═════════════════════════════════════════════════════════════════════════════
# DEMO / SYNTHETIC TERRAIN
# ═════════════════════════════════════════════════════════════════════════════

def _synthetic_distance(tilt_deg: float) -> float:
    """
    Return a plausible LIDAR distance (m) for a given tilt angle as if the
    sensor is looking at sloped ground with a raised ridge in the middle.

    Used when DEMO_MODE = True so the test can run without hardware.
    """
    # Ground model: flat at 0 m with a ridge peaking at +15 cm around 5 m ahead.
    tilt_rad = math.radians(tilt_deg)
    # At the tilt angle the sensor "sees" the ground at this forward distance.
    # When looking down the ray hits the ground (at LIDAR_HEIGHT_CM below mount).
    if tilt_rad <= 0:
        # Looking down or level: intersect ray with z = 0 (ground plane)
        forward = LIDAR_HEIGHT_CM / max(math.sin(-tilt_rad), 0.001) / 100.0
    else:
        # Looking up: point is above the ground — model a ridge
        forward = 3.0  # fixed 3 m
    # Add a synthetic ridge: ground rises by 0.15 m between 2 and 4 m ahead
    if 2.0 <= forward <= 4.0:
        ridge_height_m = 0.15 * math.sin(math.pi * (forward - 2.0) / 2.0)
    else:
        ridge_height_m = 0.0
    # Recalculate slant distance to the raised ground point
    horiz  = forward
    height = LIDAR_HEIGHT_CM / 100.0 - ridge_height_m
    return math.hypot(horiz, height)


# ═════════════════════════════════════════════════════════════════════════════
# SCAN → HEIGHTMAP
# ═════════════════════════════════════════════════════════════════════════════

def run_scan(bus=None, rover_pitch: float = 0.0) -> Heightmap:
    """
    Sweep through SCAN_ANGLES_DEG, read the LIDAR at each angle, and populate
    a Heightmap.  Each reading is projected onto the ground plane:

        d_horiz   = distance × cos(effective_tilt)   [horizontal range, m]
        dz        = distance × sin(effective_tilt)    [height offset above sensor, m]
        cell_row  = round(d_horiz / CELL_SIZE_CM × 100)
        cell_col  = MAP_SIDE_COLS   (centre column — horizontal sweep is fixed)
        height    = dz × 100  [cm]  stored in the heightmap

    effective_tilt = reported_tilt + rover_pitch  (corrects for body lean)

    When DEMO_MODE = True, the bus argument is ignored and _synthetic_distance
    is used instead of real hardware.

    Returns a populated Heightmap ready for slope computation.
    """
    total_cols = MAP_SIDE_COLS * 2 + 1
    hmap = Heightmap(MAP_FORWARD_ROWS, total_cols)
    centre_col = MAP_SIDE_COLS   # rover is at the centre column

    print(f"\n  Scanning {len(SCAN_ANGLES_DEG)} angles: "
          f"{SCAN_ANGLES_DEG[0]}° → {SCAN_ANGLES_DEG[-1]}°  "
          f"(pitch correction {rover_pitch:+.1f}°)")

    for tilt_deg in SCAN_ANGLES_DEG:
        effective_tilt = tilt_deg + rover_pitch
        t_rad = math.radians(effective_tilt)

        if DEMO_MODE or bus is None:
            distance = _synthetic_distance(tilt_deg)
        else:
            distance = _read_lidars(bus)

        # Project the range reading onto the grid
        d_horiz = distance * math.cos(t_rad)          # metres along ground
        dz      = distance * math.sin(t_rad)           # metres up from sensor

        # Convert to absolute terrain height (cm).
        # The sensor is LIDAR_HEIGHT_CM above ground, so:
        #   terrain_height = LIDAR_HEIGHT_CM - dz_cm  when looking down (dz < 0)
        # We store dz directly as a height offset so the heightmap reflects
        # shape rather than absolute altitude — consistent with main.py.
        height_cm = round(dz * 100)

        # Forward cell index (row 0 = immediately ahead of rover)
        cell_row = round((d_horiz * 100) / CELL_SIZE_CM)

        if 0 <= cell_row < MAP_FORWARD_ROWS and 0 <= centre_col < total_cols:
            hmap.heights[cell_row][centre_col] = height_cm

        print(f"    tilt={tilt_deg:+4d}°  eff={effective_tilt:+5.1f}°  "
              f"dist={distance:.3f} m  row={cell_row:3d}  height={height_cm:+d} cm")

    return hmap


# ═════════════════════════════════════════════════════════════════════════════
# GRID DISPLAY
# ═════════════════════════════════════════════════════════════════════════════

# Block characters ordered from lightest (flat) to darkest (steep)
_ELEVATION_CHARS = ["░░", "▒▒", "▓▓", "██"]
_NO_DATA         = "??"   # cell was never written by the scan


def _elevation_symbol(height_cm: int, h_min: int, h_max: int) -> str:
    """
    Map an absolute height value to one of four block characters.

        ░░  bottom quarter of the height range  (flattest / lowest)
        ▒▒  second quarter
        ▓▓  third quarter
        ██  top quarter                          (steepest / highest)

    Returns _NO_DATA if the height has not been recorded (still 0 and
    surrounded by zeros).  Cells at exactly h_min always return ░░.
    """
    if h_max == h_min:
        return _ELEVATION_CHARS[0]   # all cells flat — show lightest block
    ratio = (height_cm - h_min) / (h_max - h_min)
    index = min(int(ratio * len(_ELEVATION_CHARS)), len(_ELEVATION_CHARS) - 1)
    return _ELEVATION_CHARS[index]


def _is_obstacle(r: int, c: int, hmap: Heightmap) -> bool:
    """True when any cardinal neighbour's height diff exceeds MAX_HEIGHT_DIFF."""
    h = hmap.heights[r][c]
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < hmap.rows and 0 <= nc < hmap.cols:
            if abs(hmap.heights[nr][nc] - h) > MAX_HEIGHT_DIFF:
                return True
    return False


def print_elevation_grid(hmap: Heightmap) -> None:
    """
    Print a block grid showing the relative elevation of each heightmap cell.

    The height range across all scanned cells is divided into four quartiles,
    each mapped to a progressively denser block character:

        ░░  lowest 25 % of height values  (flattest terrain)
        ▒▒  25 – 50 %
        ▓▓  50 – 75 %
        ██  top 25 %                       (steepest / tallest terrain)
        ??  no LIDAR reading recorded for this cell

    Row 0 is closest to the rover; the last row is furthest away.
    The centre column is directly ahead; columns to the left/right are lateral.
    """
    all_heights = [
        hmap.heights[r][c]
        for r in range(hmap.rows)
        for c in range(hmap.cols)
    ]
    h_min = min(all_heights)
    h_max = max(all_heights)

    print(f"\n  Elevation grid  ({hmap.rows} rows × {hmap.cols} cols, "
          f"{CELL_SIZE_CM:.0f} cm/cell)")
    print(f"  Range: {h_min:+d} cm … {h_max:+d} cm")
    print(f"  ░░=low  ▒▒=med-low  ▓▓=med-high  ██=high  ??=no data")
    print(f"  Row 0 = closest to rover, col {hmap.cols // 2} = straight ahead\n")

    # Column index labels (every 5th column)
    header = "      "
    for c in range(hmap.cols):
        header += f"{c:2d}" if c % 5 == 0 else "  "
    print(header)

    print("    ┌" + "──" * hmap.cols + "┐")
    for r in range(hmap.rows):
        line = f"  {r:2d}│"
        for c in range(hmap.cols):
            h = hmap.heights[r][c]
            # Mark cells that were never written (still 0, neighbours also 0)
            is_unwritten = (h == 0 and all(
                hmap.heights[r + dr2][c + dc2] == 0
                for dr2, dc2 in ((-1,0),(1,0),(0,-1),(0,1))
                if 0 <= r+dr2 < hmap.rows and 0 <= c+dc2 < hmap.cols
            ))
            if is_unwritten:
                line += _NO_DATA
            else:
                line += _elevation_symbol(h, h_min, h_max)
        print(line + "│")
    print("    └" + "──" * hmap.cols + "┘\n")


def print_traversability_grid(hmap: Heightmap) -> None:
    """
    Print a block grid showing which cells are safe to traverse.

    A cell is marked as an obstacle (██) when any of its four cardinal
    neighbours has a height difference greater than MAX_HEIGHT_DIFF cm.
    This mirrors the passability check used by A* in main.py.

        ··  safe — slope to all neighbours ≤ MAX_HEIGHT_DIFF cm
        ██  obstacle — too steep to traverse safely
    """
    # Count safe vs obstacle cells for the summary line
    safe_count = sum(
        1
        for r in range(hmap.rows)
        for c in range(hmap.cols)
        if not _is_obstacle(r, c, hmap)
    )
    total = hmap.rows * hmap.cols

    print(f"  Traversability grid  (MAX_HEIGHT_DIFF = {MAX_HEIGHT_DIFF} cm)")
    print(f"  ··=safe  ██=obstacle    "
          f"{safe_count}/{total} cells passable ({100*safe_count//total}%)\n")

    print("    ┌" + "──" * hmap.cols + "┐")
    for r in range(hmap.rows):
        line = f"  {r:2d}│"
        for c in range(hmap.cols):
            line += "██" if _is_obstacle(r, c, hmap) else "··"
        print(line + "│")
    print("    └" + "──" * hmap.cols + "┘\n")


# ═════════════════════════════════════════════════════════════════════════════
# A* DEMO ON SCANNED MAP
# ═════════════════════════════════════════════════════════════════════════════

def run_astar_demo(hmap: Heightmap) -> None:
    """
    Run A* from the rover's position (row 0, centre col) to the far end of the
    scanned map (last row, centre col) and print the path overlaid on the
    traversability grid.

    Symbol legend:
        RR  rover start
        GG  goal
        ██  planned path cell
        ░░  obstacle
        ··  open traversable space
    """
    planner = AStar(hmap, diagonal=True, turn_penalty=1.0,
                    max_height_diff=MAX_HEIGHT_DIFF)

    start = (0, hmap.cols // 2)
    goal  = (hmap.rows - 1, hmap.cols // 2)

    path = planner.find_path(start, goal)

    if path is None:
        print("  A* found no path from rover to far end of scan.\n")
        return

    path_set = set(path)
    print(f"  A* path: {len(path)} cells  start={start}  goal={goal}\n")
    print("  RR=rover  GG=goal  ██=path  ░░=obstacle  ··=open\n")

    print("    ┌" + "──" * hmap.cols + "┐")
    for r in range(hmap.rows):
        line = f"  {r:2d}│"
        for c in range(hmap.cols):
            pos = (r, c)
            if pos == start:
                sym = "RR"
            elif pos == goal:
                sym = "GG"
            elif pos in path_set:
                sym = "██"
            elif _is_obstacle(r, c, hmap):
                sym = "░░"
            else:
                sym = "··"
            line += sym
        print(line + "│")
    print("    └" + "──" * hmap.cols + "┘\n")


# ═════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  COSGC Rover — Height Map Test")
    print(f"  Mode: {'DEMO (synthetic terrain)' if DEMO_MODE else 'HARDWARE (real LIDARs)'}")
    print("=" * 60)

    bus        = None
    rover_pitch = 0.0

    if not DEMO_MODE:
        try:
            import smbus2  # type: ignore
            bus = smbus2.SMBus(I2C_BUS)
            print("\nConnecting to I2C bus…")

            # Check LIDAR #1
            d1 = _read_one_lidar(bus, LIDAR_ADDR_1)
            print(f"  LIDAR #1 (0x{LIDAR_ADDR_1:02x}): "
                  f"{'OK — {d1:.3f} m'.format(d1=d1) if d1 else 'NO RESPONSE'}")

            # Check LIDAR #2
            d2 = _read_one_lidar(bus, LIDAR_ADDR_2)
            print(f"  LIDAR #2 (0x{LIDAR_ADDR_2:02x}): "
                  f"{'OK — {d2:.3f} m'.format(d2=d2) if d2 else 'NO RESPONSE'}")

            if d1 is None and d2 is None:
                print("\n  WARNING: Both LIDARs unavailable — falling back to DEMO_MODE")
                DEMO_MODE = True

            # Optional IMU for pitch compensation
            imu_ok = _init_imu(bus)
            if imu_ok:
                rover_pitch = _read_pitch(bus)
                print(f"  IMU: OK — rover pitch {rover_pitch:+.1f}°")
            else:
                print("  IMU: not responding — assuming 0° pitch")

        except ImportError:
            print("  smbus2 not installed — falling back to DEMO_MODE")
            DEMO_MODE = True
        except Exception as e:
            print(f"  I2C init failed ({e}) — falling back to DEMO_MODE")
            DEMO_MODE = True

    # ── Run scan ──────────────────────────────────────────────────────────────
    print("\n" + "─" * 60)
    print("  Running vertical scan…")
    hmap = run_scan(bus=bus, rover_pitch=rover_pitch)
    hmap.compute_slopes()

    # ── Elevation grid ────────────────────────────────────────────────────────
    print("─" * 60)
    print_elevation_grid(hmap)

    # ── Traversability grid ───────────────────────────────────────────────────
    print("─" * 60)
    print_traversability_grid(hmap)

    # ── A* path overlay ───────────────────────────────────────────────────────
    print("─" * 60)
    print("  A* path overlay\n")
    run_astar_demo(hmap)

    if bus is not None:
        try:
            bus.close()
        except Exception:
            pass

    print("Done.")

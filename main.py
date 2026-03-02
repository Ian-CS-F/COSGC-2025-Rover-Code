"""
Communication Schema:
(System, Direction, Amount)

System:
    Servo (0), Motor (1)

Direction:
    left (0), right (1), up (2), down (3)

Amount:
    float value

Example: (1, 0, 0.5) → Motor, left, 50% power
"""

import sys
import serial

sys.path.append("Control/Nav")
from heightmap import Heightmap  # type: ignore
from astar import AStar          # type: ignore

# ── Serial ────────────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 9600

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# ── Command constants ─────────────────────────────────────────────────────────
SERVO, MOTOR     = 0, 1
LEFT, RIGHT, UP, DOWN = 0, 1, 2, 3


def send_command(system: int, direction: int, amount: float) -> None:
    """Send a (System, Direction, Amount) command to the Arduino."""
    packet = f"({system},{direction},{amount:.3f})\n"
    ser.write(packet.encode())


# ── Map configuration ─────────────────────────────────────────────────────────
MAP_WIDTH_M      = 3.0   # physical width  in metres
MAP_HEIGHT_M     = 8.0   # physical height in metres
RESOLUTION_M     = 0.05    # metres per cell

COLS = int(MAP_WIDTH_M  / RESOLUTION_M)
ROWS = int(MAP_HEIGHT_M / RESOLUTION_M)


# ── Boot ──────────────────────────────────────────────────────────────────────
def main() -> None:
    heightmap = Heightmap(ROWS, COLS)
    planner   = AStar(heightmap, diagonal=True, turn_penalty=1.0, max_height_diff=2)

    print(f"Map: {ROWS} rows × {COLS} cols  ({RESOLUTION_M} m/cell)")
    print(f"Serial: {SERIAL_PORT} @ {BAUD_RATE} baud")

    while True:
        # TODO: read sensors and update heightmap.heights / heightmap.slopes
        # TODO: determine start (current position) and goal
        # TODO: path = planner.find_path(start, goal)
        # TODO: convert next step in path to motor/servo commands via send_command()
        pass


if __name__ == "__main__":
    main()

# COSGC 2025 Rover

Autonomous rover control system for the Colorado Space Grant Consortium 2025 competition.
The rover navigates an outdoor sand course, building a terrain heightmap with LIDAR and
ultrasonics while planning a path with A* around obstacles and drop-offs.

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| Main computer | Raspberry Pi | — |
| Microcontroller | Arduino (Uno/Mega) | USB Serial |
| IMU | ICM-20948 (accel + gyro + mag) | I²C (0x68) |
| LIDAR | Garmin LIDAR-Lite v4 LED | I²C (0x62) |
| Motors | L298N dual H-bridge | PWM + direction pins |
| Motor encoder | SparkFun 64 P/R quadrature | Interrupt pins 2/3 |
| Camera tilt | Servo on pin 11 | PWM |
| Ground ultrasonic | HC-SR04 (45° down) | A0/A1 |
| Sky ultrasonic | HC-SR04 (45° up) | A2/A3 |

---

## Repository Layout

```
COSGC-2025-Rover-Code/
├── Arduino/
│   └── rover_interpreeter.ino   # Arduino firmware
└── Python/
    ├── main.py                  # Raspberry Pi entry point
    ├── astar.py                 # A* path planner
    ├── heightmap.py             # Terrain heightmap grid
    └── slope.py                 # Per-cell directional slope values
```

---

## Communication Protocol

All messages are plain ASCII over 9600-baud serial between Pi and Arduino.

### Pi → Arduino

Commands follow the format `(System,Direction,Amount,Speed)\n`.

| System | ID | Direction | Amount | Speed |
|---|---|---|---|---|
| Servo | 0 | up(2) / down(3) | 0.0–1.0 → 0–180° | — |
| Motor | 1 | up(2) / down(3) / left(0) / right(1) | distance in cm (0 = continuous) | 0.0–1.0 PWM duty |
| Sweep | 2 | — | total arc in degrees | ms per tilt step |
| Query | 3 | — | — | — |
| Flip  | 4 | — | 1.0 = inverted, 0.0 = normal | — |

### Arduino → Pi

| Message | Trigger |
|---|---|
| `READY` | Sent every second on boot until Pi replies `ACK` |
| `DONE:<cm>,<ratio>` | Motor segment complete; actual encoder distance + slip ratio |
| `CLIFF:<cm>` | Ground ultrasonic lost ground; partial distance before stop |
| `AT:<h>,<t>` | Sweep position report; horizontal and tilt angles |
| `SWEEP_DONE` | Sweep finished; rover back at heading 0 |
| `DIST:<cm>` | Response to Query command; distance since last query |

---

## Key Behaviours

### Sensor Sweep
The Arduino rotates the rover left to `−range/2`, then steps right in 5° increments.
At each horizontal position the tilt servo visits a fixed set of angles
`{60°, 75°, 90°, 105°, 120°}` (up then back down), sending `AT:h,t` at each stop.
The Pi reads the LIDAR, computes the 3D target cell, and stores the height in the
heightmap before replying `NEXT`. After returning to heading 0, the Arduino sends
`SWEEP_DONE`. Pass `range_deg=360.0` for a full 360° situational-awareness sweep.

### Motor Slip Compensation
`move_rover()` sends distance segments to the Arduino and reads back the encoder
error ratio (`actual_counts / target_counts`). If the ratio deviates more than 15%
from 1.0 (wheel slip on sand), the drive speed is scaled by `1 / error_ratio` and
saved as the new default for all future segments — so the rover learns the optimal
speed for current ground conditions autonomously.

### Cliff Detection
The ground-facing ultrasonic reads continuously during driving. If the distance
suddenly exceeds `CLIFF_THRESHOLD_CM` (default 40 cm), the Arduino stops immediately
and sends `CLIFF:<partial_cm>`. The Pi catches this as a `CliffDetected` exception,
marks the cell ahead in the heightmap 5 cm below the current cell (making it
impassable to A*), performs a full 360° LIDAR sweep, and replans the path.

### Upside-Down Mode
If the rover flips, the ICM-20948 Z-axis reads ≈ −1 g. The Pi detects this, sends a
`FLIP` command to the Arduino, and toggles `_flipped`. Effects:

- **Arduino** — motor directions invert (UP↔DOWN, LEFT↔RIGHT); cliff detection
  switches from the downward to the upward ultrasonic.
- **Pi** — `read_heading()` adds 180° so dead-reckoning remains correct;
  `read_pitch()` negates so the sign convention stays consistent with effective
  forward travel.

---

## Calibration Constants

These must be measured on the physical rover before competition.

| Constant | File | Default | What to Measure |
|---|---|---|---|
| `DIST_PER_REV_CM` | `.ino` | `10.0` | Wheel circumference in cm |
| `MS_PER_DEGREE` | `.ino` | `12` | Motor runtime (ms) per degree of in-place turn |
| `CLIFF_THRESHOLD_CM` | `.ino` | `40.0` | 2× expected ground-sensor reading on flat terrain |
| `ROBOT_HEIGHT_CM` | `main.py` | `20` | Physical rover height |

---

## Setup

### Arduino
1. Open `Arduino/rover_interpreeter.ino` in the Arduino IDE.
2. Verify pin assignments match your wiring (see constants at the top of the file).
3. Upload to the board.

### Raspberry Pi
```bash
pip install pyserial smbus2
python Python/main.py
```

The Pi waits for the Arduino `READY` handshake before doing anything.
Set `SERIAL_PORT` in `main.py` if your Arduino appears on a different device
(e.g. `/dev/ttyACM0`).

---

## Path Planning

`AStar` operates on a `Heightmap` grid (default 8 m × 3 m at 5 cm/cell).
Cost per move = `|slope| + turn_penalty` (diagonal moves supported).
Cells whose height differs from a neighbour by more than `max_height_diff` (default 2 cm)
are treated as impassable.

The rover starts at the middle-bottom cell `(ROWS−1, COLS//2)` and dead-reckons
its position each loop using encoder distance + tilt-compensated compass heading.

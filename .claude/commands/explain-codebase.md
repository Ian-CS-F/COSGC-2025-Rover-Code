## COSGC 2025 Rover — Codebase Overview

This is an autonomous rover for the Colorado Space Grant Consortium 2025 competition. The rover must navigate an outdoor sand course from a start position to a goal, avoiding obstacles and drop-offs, without human input.

---

### Two-Processor Architecture

The system is split across two processors that communicate over USB serial at 9600 baud:

- **Raspberry Pi** (`Python/main.py`) — high-level brain. Handles LIDAR, IMU, path planning, and decision-making.
- **Arduino** (`Arduino/rover_interpreter.ino`) — low-level executor. Drives motors, reads ultrasonics, controls the tilt servo.

---

### Serial Protocol

All messages are plain ASCII. Commands go Pi → Arduino, responses go Arduino → Pi.

**Pi → Arduino** — format: `(<system>,<direction>,<amount>,<speed>)\n`

| System | ID | What it does |
|---|---|---|
| Servo | 0 | Set tilt servo angle. amount = 0.0–1.0 → 0–180° |
| Motor | 1 | Drive motors. direction = UP/DOWN/LEFT/RIGHT. amount = distance in cm (0 = continuous). speed = 0.0–1.0 PWM |
| Sweep | 2 | Perform a LIDAR sweep. amount = total arc in degrees. speed = ms per tilt step |
| Query | 3 | Ask for encoder distance since last query |
| Flip  | 4 | amount = 1.0 → inverted mode, 0.0 → normal mode |

**Motor direction note:** LEFT and RIGHT always run continuously and never send DONE — the Arduino returns immediately after starting the motors. Only UP and DOWN with a nonzero distance send DONE.

**Arduino → Pi** responses:

| Message | When sent |
|---|---|
| `READY` | Every second on boot until Pi replies `ACK` |
| `DONE:<cm>,<ratio>` | Motor UP/DOWN segment complete. actual encoder distance + (actual_counts/target_counts) |
| `CLIFF:<cm>` | Ultrasonic lost ground mid-drive. Motors already stopped. Partial encoder distance |
| `AT:<h>,<t>` | Sweep position: horizontal angle and tilt offset from 90° |
| `SWEEP_DONE` | Sweep finished, rover back at heading 0 |
| `DIST:<cm>` | Response to Query. Encoder distance since last query, then resets |

**Handshake:** Arduino sends `READY` every second. Pi reads until it sees `READY`, then sends `ACK`. After that, commands can be sent.

---

### Sensors

**Garmin LIDAR-Lite v4 LED** — I2C at address `0x62` on bus 1. To take a reading: write `0x04` to register `0x00`, wait 20 ms, read high byte from `0x0f` and low byte from `0x10`, combine as `((high << 8) | low) / 100.0` to get metres. Used during sweeps to measure terrain distance at each angle.

**HC-SR04 Ultrasonics** — Two sensors on the Arduino. Downward-facing on A0/A1 for normal mode cliff detection, upward-facing on A2/A3 for inverted mode. Standard trigger/echo pulse protocol. Returns distance in cm; 999.0 if no echo. Cliff threshold is 40 cm — if exceeded mid-drive, motors stop and `CLIFF:<cm>` is sent.

**ICM-20948 IMU** — I2C at `0x68`. Provides compass heading (tilt-compensated using accelerometer + magnetometer via AK09916 at `0x0C`), pitch angle, and Z-axis acceleration for flip detection. Used for turning to correct headings and dead-reckoning position.

---

### Navigation Pipeline

1. **Sweep** — Arduino rotates rover left to −range/2, then steps right in 5° increments. At each horizontal position the tilt servo visits `{60°, 75°, 90°, 105°, 120°}` up then back down. Arduino sends `AT:<h>,<t>` at each stop; Pi reads LIDAR, computes the 3D point, stores height in the heightmap, sends `NEXT`.

2. **Heightmap** — 2D grid (default 8 m × 3 m at 5 cm/cell = 160 rows × 60 cols). Each cell stores terrain height in cm. `compute_slopes()` calculates per-cell slope values used as A* movement cost.

3. **A\*** — Plans a path from current grid cell to goal cell `(0, COLS//2)`. Cost per move = slope + turn penalty. Cells differing from a neighbour by more than `max_height_diff` (2 cm) are impassable. Replans every 5 path steps or after any cliff event.

4. **Turn** — `turn_to_heading()` sends continuous LEFT or RIGHT motor commands and polls IMU heading until within 5° of target. Then sends speed=0 to stop.

5. **Drive** — `move_rover()` sends distance-controlled UP segments to the Arduino and reads `DONE:<cm>,<ratio>` back. If `error_ratio` deviates more than 15% from 1.0 (slip on sand), drive speed is scaled by `1 / error_ratio` and saved for future segments — adaptive slip compensation.

6. **Dead-reckon** — After each segment, `update_position()` advances the grid cell using encoder distance + IMU compass heading relative to the initial heading at startup.

---

### Cliff Handling

When `CLIFF:<cm>` arrives (from mid-drive ultrasonic or idle 1 Hz check), the Pi:
1. Checks IMU pitch — if the rover is climbing a slope (pitch > 30°), it's a false alarm.
2. Marks the cell directly ahead in the heightmap 5 cm below current height (making it impassable).
3. Does a full 360° LIDAR sweep to refresh the map.
4. Replans with A*.

---

### Flip Handling

The ICM-20948 Z-axis reads ≈ +1 g upright and ≈ −1 g inverted. When it crosses −0.5 g, the Pi sends `(4, 0, 1.0, 0)` to the Arduino.

- **Arduino**: inverts all motor directions (UP↔DOWN, LEFT↔RIGHT); switches cliff sensor from downward to upward ultrasonic.
- **Pi**: `read_heading()` adds 180° so dead-reckoning stays correct; `read_pitch()` negates so sign convention stays consistent with forward travel.

---

### File Map

| File | Role |
|---|---|
| `Python/main.py` | Entry point. All serial, LIDAR, and IMU functions. Navigation orchestration. |
| `Arduino/rover_interpreter.ino` | Firmware. Motor control, encoder ISR, ultrasonic cliff detection, servo sweep, serial parser. |
| `Python/Control/pid.py` | Simple PID class: `PID(kp, ki, kd)` / `update(target, measured, dt)`. Used conceptually for slip compensation. |
| `Python/Hardware/motor_interface.py` | Abstract `MotorInterface` base class (for simulation vs real motor swapping). |
| `Python/Control/Nav/astar.py` | A* path planner operating on a Heightmap grid. |
| `Python/Control/Nav/heightmap.py` | 2D height grid with `compute_slopes()`. |
| `Python/Control/Nav/slope.py` | Per-cell directional slope calculations. |
| `Test Protocols/` | Standalone hardware tests for each subsystem. |

---

### Constants That Must Be Calibrated on the Physical Rover

| Constant | File | What to measure |
|---|---|---|
| `DIST_PER_REV_CM` | `.ino` | Mark the ground, drive one encoder revolution, measure distance |
| `MS_PER_DEGREE` | `.ino` | Time a 90° in-place turn, divide by 90 |
| `CLIFF_THRESHOLD_CM` | `.ino` | 2× the ultrasonic reading on flat ground at mounted height |
| `TURN_TOLERANCE_DEG` | `main.py` | Minimum acceptable heading error before stopping a turn |

# COSGC 2026 Rover

Autonomous rover control system for the Colorado Space Grant Consortium 2026 competition.
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
COSGC-2026-Rover-Code/
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

## Computer Development Mode (No Raspberry Pi Required)

This lets any team member drive the rover and test the navigation code from a
Windows, Mac, or Linux laptop. You do not need a Raspberry Pi.

There are three levels depending on what hardware you have available:

| Level | Hardware needed | What you can test |
|---|---|---|
| **Full** | Motor Arduino + Sensor Arduino + IMU + LIDAR wired up | Everything |
| **Motors only** | Motor Arduino connected via USB | Driving, encoder, cliff detection |
| **No hardware** | Nothing | Path planning and navigation logic only |

Start with Level 1 (No hardware) to make sure the code runs on your machine,
then move up as you get access to hardware.

---

### Prerequisites

Before anything else, install these on your laptop:

1. **Python 3.10 or newer** — download from [python.org](https://python.org).
   Verify with `python --version` (or `python3 --version` on Mac/Linux).

2. **Arduino IDE 2.x** — download from [arduino.cc](https://www.arduino.cc/en/software).
   Needed to upload firmware to the Arduinos.

3. **Python packages** — open a terminal and run:
   ```bash
   pip install pyserial pynput
   ```
   On Mac/Linux you may need `pip3` instead of `pip`.

---

### Level 1 — No Hardware (test navigation logic only)

No Arduinos, no sensors, no wiring. Everything is simulated in software.

1. Open `Python/computer_main.py` in a text editor.

2. At the very top of the file, make sure this line reads `False`:
   ```python
   HARDWARE_MODE = False
   ```

3. From the repo root, run:
   ```bash
   python Python/computer_main.py
   ```

4. When prompted, press **Enter** to start keyboard mode.

You will see the live display and can press W/A/S/D — motor commands are
printed to the terminal instead of sent to real hardware. Sensor values are
fixed (heading = 0°, LIDAR = 2 m). This is enough to verify the code runs
correctly on your machine before touching any hardware.

---

### Level 2 — Motors Only

You need: one Arduino (Uno or Mega) running the rover firmware, connected via USB.

#### Step 1 — Upload firmware to the motor Arduino

1. Open the Arduino IDE.
2. Go to **File → Open** and select `Arduino/rover_interpreter.ino`.
3. Plug the Arduino into your laptop via USB.
4. In the Arduino IDE, go to **Tools → Board** and select your board
   (Arduino Uno or Arduino Mega 2560).
5. Go to **Tools → Port** and select the port that appeared when you plugged
   in the Arduino (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux,
   `/dev/cu.usbmodem...` on Mac).
6. Click the **Upload** button (→ arrow). Wait for "Done uploading."

#### Step 2 — Find the port name

You need the exact port name to put in the Python config. The easiest way:

- **Windows**: Open Device Manager → expand "Ports (COM & LPT)" → look for
  "Arduino" or "USB Serial Device" → note the `COM` number (e.g. `COM4`).
- **Mac**: Open a terminal and run `ls /dev/cu.*` before and after plugging in —
  the new entry is your port (e.g. `/dev/cu.usbmodem14201`).
- **Linux**: Open a terminal and run `ls /dev/ttyUSB* /dev/ttyACM*` before and
  after plugging in — the new entry is your port (e.g. `/dev/ttyUSB0`).

#### Step 3 — Configure and run

1. Open `Python/computer_main.py` and set these two lines at the top:
   ```python
   HARDWARE_MODE = True
   MOTOR_PORT    = "COM4"          # replace with your actual port
   ```
   Leave `SENSOR_PORT` as-is for now — it is not used in motors-only mode.

2. Run:
   ```bash
   python Python/computer_main.py
   ```

3. You should see `Motor Arduino connected.` If you see
   `RuntimeError: Motor Arduino did not send READY`, the port is wrong or the
   firmware did not upload — go back to Step 1.

#### Step 4 — Verify with keyboard mode

Press Enter to start keyboard mode. Hold **W** — the rover should drive forward.
The encoder distance in the display should increase. Press **Q** to stop.

---

### Level 3 — Full Hardware (motors + IMU + LIDAR)

You need: the motor Arduino (from Level 2) **plus** a second Arduino with the
IMU and LIDAR wired to it.

#### Step 1 — Wire the sensor Arduino

The second Arduino reads both sensors over I2C and streams the data to the
laptop. Wire it as follows:

| Sensor pin | Arduino pin | Notes |
|---|---|---|
| ICM-20948 SDA | A4 (Uno) or pin 20 (Mega) | Shared with LIDAR |
| ICM-20948 SCL | A5 (Uno) or pin 21 (Mega) | Shared with LIDAR |
| ICM-20948 VCC | 3.3 V | Do **not** use 5 V — will damage the chip |
| ICM-20948 GND | GND | |
| **ICM-20948 AD0** | **3.3 V** | **Required** — sets I2C address to 0x69 |
| LIDAR-Lite v4 SDA | A4 (Uno) or pin 20 (Mega) | Same pins as IMU |
| LIDAR-Lite v4 SCL | A5 (Uno) or pin 21 (Mega) | Same pins as IMU |
| LIDAR-Lite v4 VCC | 5 V | Needs up to 1 A — use a powered USB hub if the Arduino resets on startup |
| LIDAR-Lite v4 GND | GND | |

Both sensors share the same SDA and SCL wires. This is normal — I2C is a
shared bus and each sensor has a unique address (IMU = 0x69, LIDAR = 0x62).

#### Step 2 — Upload firmware to the sensor Arduino

1. In the Arduino IDE, go to **File → Open** and select
   `Arduino/i2c_bridge/i2c_bridge.ino`.
2. Plug the **sensor** Arduino into the laptop (the motor Arduino can stay
   connected on its own port).
3. Go to **Tools → Board** and select your board.
4. Go to **Tools → Port** and select the sensor Arduino's port.
5. Click **Upload**. Wait for "Done uploading."

#### Step 3 — Verify the sensor Arduino

1. In the Arduino IDE, go to **Tools → Serial Monitor**.
2. Set the baud rate to **9600** (bottom-right dropdown).
3. You should see lines like this arriving every 100 ms:
   ```
   IMU:0.0123,-0.0045,0.9987,0.12,-0.03,0.01,142,-23,88
   LIDAR:1.4200
   ```
   If you see `ERROR:IMU` — check that AD0 is wired to 3.3 V and VCC is 3.3 V.
   If you see `ERROR:LIDAR` — check power (LIDAR needs ~1 A; try a powered hub).

   Close the Serial Monitor before running Python — only one program can use
   the serial port at a time.

#### Step 4 — Configure and run

1. Note the sensor Arduino's port name (same method as Level 2 Step 2).

2. Open `Python/computer_main.py` and set all three lines:
   ```python
   HARDWARE_MODE = True
   MOTOR_PORT    = "COM4"           # your motor Arduino port
   SENSOR_PORT   = "COM5"           # your sensor Arduino port
   ```

3. Run:
   ```bash
   python Python/computer_main.py
   ```

4. You should see both arduinos connect and live sensor data in the display.

---

### Keyboard Controls

Once in keyboard mode, the terminal shows a live status display:

```
=== COSGC Rover  |  KEYBOARD mode  |  HW: REAL ===
  Heading :  127.3 deg     Pitch  :   2.1 deg
  LiDAR   :    1.43 m      Encoder:  12.4 cm
  Speed   :   0.50         Driving: FORWARD

  W/S/A/D or arrows = drive    +/- = speed
  SPACE = toggle auto/keyboard  Q/ESC = quit
```

| Key | Action |
|---|---|
| W or ↑ | Drive forward |
| S or ↓ | Drive reverse |
| A or ← | Tank-turn left |
| D or → | Tank-turn right |
| + or = | Speed up (max 1.0) |
| - | Slow down (min 0.1) |
| Space | Switch to autonomous mode (press again to return) |
| Q or ESC | Stop and quit |

---

### Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| `python: command not found` | Python not installed or not on PATH | Install Python 3.10+; on Mac/Linux try `python3` |
| `ModuleNotFoundError: No module named 'serial'` | pyserial not installed | `pip install pyserial` |
| `ModuleNotFoundError: No module named 'pynput'` | pynput not installed | `pip install pynput` |
| `RuntimeError: Motor Arduino did not send READY` | Wrong port or wrong firmware | Check `MOTOR_PORT`; re-upload `rover_interpreter.ino` |
| `WARNING: IMU not responding` | AD0 not wired to 3.3 V, or wrong firmware | Check AD0; re-upload `i2c_bridge.ino` |
| `WARNING: LIDAR not responding` | Power — LIDAR draws ~1 A at startup | Use a powered USB hub for the sensor Arduino |
| `ERROR:IMU` in Serial Monitor | I2C wiring issue or missing pull-ups | Check SDA/SCL connections; some modules need 4.7 kΩ pull-ups to 3.3 V |
| Arduino IDE shows no port | Driver not installed | Windows: install CH340 or CP210x driver; Mac: install per Arduino IDE prompt |
| Serial Monitor open when running Python | Only one program can own a port | Close Serial Monitor before running `computer_main.py` |

---

## Path Planning

`AStar` operates on a `Heightmap` grid (default 8 m × 3 m at 5 cm/cell).
Cost per move = `|slope| + turn_penalty` (diagonal moves supported).
Cells whose height differs from a neighbour by more than `max_height_diff` (default 2 cm)
are treated as impassable.

The rover starts at the middle-bottom cell `(ROWS−1, COLS//2)` and dead-reckons
its position each loop using encoder distance + tilt-compensated compass heading.

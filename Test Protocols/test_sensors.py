"""
Test Protocol: Sensors
Streams live readings from each sensor for 10 seconds each.
  - LIDAR #1 (0x5b)
  - LIDAR #2 (0x62)
  - Both LIDARs averaged
  - IMU accelerometer + gyroscope
  - IMU magnetometer + compass heading
  - Ultrasonic down (cliff sensor, normal mode)
  - Ultrasonic up  (cliff sensor, flipped mode)

Ultrasonics are read by the Arduino Mega, not the Pi directly.
The Arduino sends CLIFF:<left_cm>,<right_cm> unsolicited at 1 Hz when
a sensor loses the ground. During the ultrasonic test you will be prompted
to cover each sensor with your hand to trigger a CLIFF message.

Run on the Pi:
    python3 "test_sensors.py"
"""

import math
import struct
import time
import smbus2
import serial as pyserial

# ── Config ────────────────────────────────────────────────────────────────────
I2C_BUS      = 1
LIDAR_ADDR_1 = 0x5b
LIDAR_ADDR_2 = 0x62
ICM_ADDR     = 0x69
MAG_ADDR     = 0x0C

SERIAL_PORT  = "/dev/ttyACM0"  # Arduino Mega on Pi
BAUD_RATE    = 9600

TEST_DURATION = 10   # seconds per sensor block
POLL_INTERVAL = 0.2  # seconds between readings

# ── ICM-20948 registers ───────────────────────────────────────────────────────
REG_BANK_SEL  = 0x7F
PWR_MGMT_1    = 0x06
INT_PIN_CFG   = 0x0F
ACCEL_XOUT_H  = 0x2D   # 6 bytes accel + 6 bytes gyro contiguous
GYRO_XOUT_H   = 0x33
MAG_CNTL2     = 0x31
MAG_ST1       = 0x10
MAG_HXL       = 0x11

ACCEL_SCALE = 16384.0  # ±2 g
GYRO_SCALE  =   131.0  # ±250 dps


# ── Helpers ───────────────────────────────────────────────────────────────────
def icm_bank(bus, bank):
    bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, bank << 4)


def init_imu(bus):
    icm_bank(bus, 0)
    bus.write_byte_data(ICM_ADDR, PWR_MGMT_1,  0x01)  # wake, auto clock
    time.sleep(0.1)
    bus.write_byte_data(ICM_ADDR, INT_PIN_CFG, 0x02)  # I2C bypass for mag
    time.sleep(0.02)
    bus.write_byte_data(MAG_ADDR, MAG_CNTL2,   0x08)  # AK09916 100 Hz continuous
    time.sleep(0.02)


def read_one_lidar(bus, addr):
    """Returns distance in metres, or None on error."""
    try:
        bus.write_byte_data(addr, 0x00, 0x04)
        time.sleep(0.02)
        high = bus.read_byte_data(addr, 0x0f)
        low  = bus.read_byte_data(addr, 0x10)
        return ((high << 8) | low) / 100.0
    except OSError:
        return None


def read_imu(bus):
    """Returns (ax, ay, az, gx, gy, gz, mx, my, mz) or None on error."""
    try:
        icm_bank(bus, 0)
        raw = bus.read_i2c_block_data(ICM_ADDR, ACCEL_XOUT_H, 12)
        ax = struct.unpack('>h', bytes(raw[0:2]))[0]  / ACCEL_SCALE
        ay = struct.unpack('>h', bytes(raw[2:4]))[0]  / ACCEL_SCALE
        az = struct.unpack('>h', bytes(raw[4:6]))[0]  / ACCEL_SCALE
        gx = struct.unpack('>h', bytes(raw[6:8]))[0]  / GYRO_SCALE
        gy = struct.unpack('>h', bytes(raw[8:10]))[0] / GYRO_SCALE
        gz = struct.unpack('>h', bytes(raw[10:12]))[0]/ GYRO_SCALE

        mx = my = mz = None
        st1 = bus.read_byte_data(MAG_ADDR, MAG_ST1)
        if st1 & 0x01:
            mb = bus.read_i2c_block_data(MAG_ADDR, MAG_HXL, 8)
            mx = struct.unpack('<h', bytes(mb[0:2]))[0]
            my = struct.unpack('<h', bytes(mb[2:4]))[0]
            mz = struct.unpack('<h', bytes(mb[4:6]))[0]

        return ax, ay, az, gx, gy, gz, mx, my, mz
    except OSError:
        return None


def heading_from_mag(ax, ay, az, mx, my, mz):
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    mx_c  = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_c  = (mx * math.sin(roll) * math.sin(pitch)
             + my * math.cos(roll)
             - mz * math.sin(roll) * math.cos(pitch))
    return math.degrees(math.atan2(-my_c, mx_c)) % 360


def run_for(seconds, label, fn):
    print(f"\n{'='*55}")
    print(f"  {label}  ({seconds}s)")
    print(f"{'='*55}")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        fn()
        time.sleep(POLL_INTERVAL)


# ── Test blocks ───────────────────────────────────────────────────────────────
def test_lidar1(bus):
    def _read():
        d = read_one_lidar(bus, LIDAR_ADDR_1)
        if d is None:
            print("  LIDAR #1 (0x5b): ERROR - no response")
        else:
            print(f"  LIDAR #1 (0x5b): {d:.3f} m  ({d*100:.1f} cm)")
    run_for(TEST_DURATION, "LIDAR #1  (0x5b)", _read)


def test_lidar2(bus):
    def _read():
        d = read_one_lidar(bus, LIDAR_ADDR_2)
        if d is None:
            print("  LIDAR #2 (0x62): ERROR - no response")
        else:
            print(f"  LIDAR #2 (0x62): {d:.3f} m  ({d*100:.1f} cm)")
    run_for(TEST_DURATION, "LIDAR #2  (0x62)", _read)


def test_lidar_averaged(bus):
    def _read():
        d1 = read_one_lidar(bus, LIDAR_ADDR_1)
        d2 = read_one_lidar(bus, LIDAR_ADDR_2)
        s1 = f"{d1:.3f} m" if d1 is not None else "ERR"
        s2 = f"{d2:.3f} m" if d2 is not None else "ERR"
        if d1 is not None and d2 is not None:
            diff = abs(d1 - d2) * 100
            avg  = (d1 + d2) / 2.0
            print(f"  #1={s1}  #2={s2}  diff={diff:.1f}cm  avg={avg:.3f} m")
        else:
            print(f"  #1={s1}  #2={s2}")
    run_for(TEST_DURATION, "Both LIDARs averaged", _read)


def test_imu_accel_gyro(bus):
    def _read():
        result = read_imu(bus)
        if result is None:
            print("  IMU: ERROR - no response")
            return
        ax, ay, az, gx, gy, gz, *_ = result
        print(f"  accel  ax={ax:+.3f}g  ay={ay:+.3f}g  az={az:+.3f}g"
              f"    gyro  gx={gx:+.1f}°/s  gy={gy:+.1f}°/s  gz={gz:+.1f}°/s")
    run_for(TEST_DURATION, "IMU — Accelerometer + Gyroscope", _read)


def test_imu_mag(bus):
    last_heading = [None]

    def _read():
        result = read_imu(bus)
        if result is None:
            print("  IMU: ERROR - no response")
            return
        ax, ay, az, _, _, _, mx, my, mz = result
        if mx is None:
            print("  Mag: waiting for data-ready...")
            return
        heading = heading_from_mag(ax, ay, az, mx, my, mz)
        last_heading[0] = heading
        print(f"  mag  mx={mx:6d}  my={my:6d}  mz={mz:6d}    heading={heading:.1f}°")

    run_for(TEST_DURATION, "IMU — Magnetometer + Heading", _read)


# ── Arduino serial helpers ────────────────────────────────────────────────────
def connect_arduino():
    """Open serial port and complete READY/ACK handshake. Returns Serial or None."""
    try:
        ser = pyserial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line == "READY":
                ser.write(b"ACK\n")
                print("  Arduino handshake OK")
                return ser
        print("  Arduino did not send READY within 10s")
        return None
    except pyserial.SerialException as e:
        print(f"  Serial open failed: {e}")
        return None


def send_command(ser, system, direction, amount, speed):
    ser.write(f"({system},{direction},{amount:.3f},{speed:.3f})\n".encode())


def drain_lines(ser, timeout=0.05):
    """Read and discard any pending lines."""
    ser.timeout = timeout
    while ser.readline():
        pass
    ser.timeout = 1


# ── Arduino test blocks ───────────────────────────────────────────────────────
def test_ultrasonic_down(ser):
    """
    Reads the downward cliff sensor by sending a QUERY and watching for CLIFF.
    The Arduino sends CLIFF unsolicited at 1 Hz when the sensor loses the ground.
    Prompt the user to cover the sensor to trigger a reading.
    """
    print(f"\n{'='*55}")
    print(f"  Ultrasonic DOWN (cliff normal mode)  ({TEST_DURATION}s)")
    print(f"{'='*55}")
    print("  Point the rover so the DOWN sensor has a clear floor view.")
    print("  Hold your hand ~50 cm above it to trigger a CLIFF reading.")

    # Put rover in normal (non-flipped) mode
    send_command(ser, 4, 0, 0.0, 0)  # FLIP off
    drain_lines(ser)

    deadline = time.monotonic() + TEST_DURATION
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("CLIFF:"):
            parts = line[6:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            print(f"  CLIFF detected  left={left_cm:.1f}cm  right={right_cm:.1f}cm")
        else:
            print(f"  [{line}]")


def test_ultrasonic_up(ser):
    """
    Same as above but switches the rover to flipped mode so the Arduino
    uses the upward sensor for cliff detection.
    """
    print(f"\n{'='*55}")
    print(f"  Ultrasonic UP (cliff flipped mode)  ({TEST_DURATION}s)")
    print(f"{'='*55}")
    print("  The rover is now in FLIPPED mode — UP sensor is active.")
    print("  Hold your hand above the UP sensor to trigger a CLIFF reading.")

    send_command(ser, 4, 0, 1.0, 0)  # FLIP on
    drain_lines(ser)

    deadline = time.monotonic() + TEST_DURATION
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("CLIFF:"):
            parts = line[6:].split(",")
            left_cm  = float(parts[0])
            right_cm = float(parts[1]) if len(parts) > 1 else left_cm
            print(f"  CLIFF detected  left={left_cm:.1f}cm  right={right_cm:.1f}cm")
        else:
            print(f"  [{line}]")

    # Restore normal mode
    send_command(ser, 4, 0, 0.0, 0)


def test_encoders(ser):
    """
    Spins each motor briefly at low speed then queries the encoder counts.
    Tests left and right independently so you can verify each encoder fires.
    """
    print(f"\n{'='*55}")
    print(f"  Encoders (left + right)")
    print(f"{'='*55}")

    DRIVE_SPEED = 0.4
    DRIVE_CM    = 20.0
    MOTOR, UP, DOWN, LEFT, RIGHT = 1, 2, 3, 0, 1

    for label, direction in [("LEFT motor / LEFT encoder", LEFT),
                              ("RIGHT motor / RIGHT encoder", RIGHT),
                              ("BOTH forward", UP),
                              ("BOTH reverse", DOWN)]:
        print(f"\n  >> {label} ({DRIVE_CM:.0f} cm at speed {DRIVE_SPEED})")
        drain_lines(ser)
        send_command(ser, MOTOR, direction, DRIVE_CM, DRIVE_SPEED)

        # Wait for DONE or timeout
        deadline = time.monotonic() + 15
        done = False
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("DONE:"):
                parts = line[5:].split(",")
                left_cm     = float(parts[0])
                right_cm    = float(parts[1]) if len(parts) > 1 else left_cm
                left_ratio  = float(parts[2]) if len(parts) > 2 else 1.0
                right_ratio = float(parts[3]) if len(parts) > 3 else 1.0
                print(f"     left={left_cm:.1f}cm (ratio={left_ratio:.3f})  "
                      f"right={right_cm:.1f}cm (ratio={right_ratio:.3f})")
                done = True
                break
            elif line.startswith("CLIFF:"):
                print(f"     CLIFF mid-drive: {line}")
                break
        if not done:
            print("     TIMEOUT — no DONE received")

        time.sleep(0.5)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Initialising I2C bus...")
    bus = smbus2.SMBus(I2C_BUS)

    print("Initialising IMU...")
    try:
        init_imu(bus)
        print("  IMU OK")
    except OSError as e:
        print(f"  IMU init FAILED: {e}")

    try:
        test_lidar1(bus)
        test_lidar2(bus)
        test_lidar_averaged(bus)
        test_imu_accel_gyro(bus)
        test_imu_mag(bus)
    finally:
        bus.close()

    print("\nConnecting to Arduino for ultrasonic + encoder tests...")
    ser = connect_arduino()
    if ser:
        try:
            test_ultrasonic_down(ser)
            test_ultrasonic_up(ser)
            test_encoders(ser)
        finally:
            ser.close()
    else:
        print("Skipping Arduino tests — could not connect.")

    print("\nDone.")

"""
sensor_bridge.py
Connects to an Arduino running i2c_bridge.ino over USB serial and provides
read_imu() / read_lidar() as a drop-in replacement for Pi native I2C reads.

The Arduino streams one IMU line and one LIDAR line every 100 ms:
    IMU:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>
    LIDAR:<dist_m>
    ERROR:IMU    ← sensor unavailable
    ERROR:LIDAR  ← sensor unavailable

Usage:
    import sensor_bridge
    sensor_bridge.connect("COM3")          # Windows
    sensor_bridge.connect("/dev/ttyUSB1")  # Linux / Mac

    imu  = sensor_bridge.read_imu()    # dict or None
    dist = sensor_bridge.read_lidar()  # float (metres) or None
    sensor_bridge.disconnect()
"""

import threading
import time
import serial


# ── Internal state ────────────────────────────────────────────────────────────
_ser      = None
_thread   = None
_running  = False
_lock     = threading.Lock()

_latest_imu   = None   # dict: ax ay az gx gy gz mx my mz
_latest_lidar = None   # float (metres)
_imu_error    = False  # True if last IMU line was ERROR:IMU
_lidar_error  = False  # True if last LIDAR line was ERROR:LIDAR


# ── Public API ────────────────────────────────────────────────────────────────
def connect(port: str, baud: int = 9600, timeout: float = 5.0) -> None:
    """
    Open the serial port and start the background reader thread.
    Blocks until the first IMU sample arrives or timeout expires.

    Args:
        port:    Serial port name (e.g. 'COM3', '/dev/ttyUSB1').
        baud:    Baud rate — must match i2c_bridge.ino (default 9600).
        timeout: Seconds to wait for the first sample before returning.
    """
    global _ser, _thread, _running

    _ser = serial.Serial(port, baud, timeout=0.2)
    _ser.reset_input_buffer()
    _running = True

    _thread = threading.Thread(target=_reader, daemon=True)
    _thread.start()

    # Wait for at least one good IMU sample so callers don't get None immediately
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with _lock:
            if _latest_imu is not None or _imu_error:
                return
        time.sleep(0.05)


def disconnect() -> None:
    """Stop the background thread and close the serial port."""
    global _running, _ser
    _running = False
    if _thread is not None:
        _thread.join(timeout=1.0)
    if _ser and _ser.is_open:
        _ser.close()


def read_imu() -> dict | None:
    """
    Return the most recent IMU sample as a dict, or None if unavailable.

    Keys (all floats unless noted):
        ax, ay, az  — acceleration in g
        gx, gy, gz  — angular rate in degrees/second
        mx, my, mz  — magnetometer raw counts (int)
    """
    with _lock:
        return dict(_latest_imu) if _latest_imu is not None else None


def read_lidar() -> float | None:
    """
    Return the most recent LIDAR distance in metres, or None if unavailable.
    """
    with _lock:
        return _latest_lidar


def is_imu_ok() -> bool:
    """True if the last IMU read succeeded (no ERROR:IMU from Arduino)."""
    with _lock:
        return not _imu_error and _latest_imu is not None


def is_lidar_ok() -> bool:
    """True if the last LIDAR read succeeded."""
    with _lock:
        return not _lidar_error and _latest_lidar is not None


# ── Background reader thread ──────────────────────────────────────────────────
def _reader() -> None:
    global _latest_imu, _latest_lidar, _imu_error, _lidar_error

    while _running:
        try:
            raw = _ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
        except Exception:
            continue

        if line.startswith("IMU:"):
            parsed = _parse_imu(line[4:])
            with _lock:
                if parsed is not None:
                    _latest_imu = parsed
                    _imu_error  = False
                else:
                    _imu_error = True

        elif line.startswith("LIDAR:"):
            try:
                dist = float(line[6:])
                with _lock:
                    _latest_lidar = dist
                    _lidar_error  = False
            except ValueError:
                with _lock:
                    _lidar_error = True

        elif line == "ERROR:IMU":
            with _lock:
                _imu_error = True

        elif line == "ERROR:LIDAR":
            with _lock:
                _lidar_error = True


def _parse_imu(payload: str) -> dict | None:
    """Parse 'ax,ay,az,gx,gy,gz,mx,my,mz' into a dict. Returns None on error."""
    try:
        parts = payload.split(",")
        if len(parts) != 9:
            return None
        return {
            "ax": float(parts[0]),
            "ay": float(parts[1]),
            "az": float(parts[2]),
            "gx": float(parts[3]),
            "gy": float(parts[4]),
            "gz": float(parts[5]),
            "mx": int(parts[6]),
            "my": int(parts[7]),
            "mz": int(parts[8]),
        }
    except (ValueError, IndexError):
        return None

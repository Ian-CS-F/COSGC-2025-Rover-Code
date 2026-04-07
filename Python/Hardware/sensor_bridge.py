"""
sensor_bridge.py
Reads the ICM-20948 IMU and two Garmin LIDAR-Lite v4s directly over the
Raspberry Pi I2C bus (bus 1, GPIO 2/3).

LIDARs are pre-configured to different addresses (0x5b and 0x62) so both
can stay powered on the same bus simultaneously.
Readings from both are averaged (or the closer one is used if they diverge).

Usage:
    import sensor_bridge
    sensor_bridge.connect()          # uses defaults
    sensor_bridge.connect(i2c_bus=1, imu_addr=0x68)

    imu  = sensor_bridge.read_imu()    # dict or None
    dist = sensor_bridge.read_lidar()  # float (metres) or None
    sensor_bridge.disconnect()
"""

import struct
import threading
import time

import smbus2

# ── Default hardware config ───────────────────────────────────────────────────
_DEFAULT_I2C_BUS  = 1
_DEFAULT_IMU_ADDR = 0x69   # ICM-20948 AD0 pulled high
_LIDAR_ADDR_1     = 0x5b   # LIDAR #1 pre-configured address
_LIDAR_ADDR_2     = 0x62   # LIDAR #2 pre-configured address

_LIDAR_TOLERANCE_CM = 10   # max divergence before discarding the farther reading

# ── ICM-20948 register map (bank 0) ──────────────────────────────────────────
_REG_BANK_SEL  = 0x7F
_PWR_MGMT_1    = 0x06
_INT_PIN_CFG   = 0x0F
_ACCEL_XOUT_H  = 0x2D   # 6 bytes: ax_h ax_l ay_h ay_l az_h az_l
_GYRO_XOUT_H   = 0x33   # 6 bytes: gx_h gx_l gy_h gy_l gz_h gz_l

# AK09916 magnetometer (exposed via I2C bypass)
_MAG_ADDR   = 0x0C
_MAG_CNTL2  = 0x31   # write 0x08 → 100 Hz continuous
_MAG_ST1    = 0x10   # bit 0 = data ready
_MAG_HXL    = 0x11   # 8 bytes: xl xh yl yh zl zh tmps st2

_ACCEL_SCALE = 16384.0   # ±2 g
_GYRO_SCALE  =   131.0   # ±250 dps

# ── Internal state ────────────────────────────────────────────────────────────
_bus         = None
_imu_addr    = _DEFAULT_IMU_ADDR
_thread      = None
_running     = False
_lock        = threading.Lock()

_latest_imu   = None   # dict: ax ay az gx gy gz mx my mz
_latest_lidar = None   # float (metres), averaged across both sensors
_imu_error    = False
_lidar_error  = False

# last good magnetometer reading (mag updates at 100 Hz, accel/gyro faster)
_last_mx, _last_my, _last_mz = 0, 0, 0


# ── Public API ────────────────────────────────────────────────────────────────
def connect(i2c_bus: int = _DEFAULT_I2C_BUS,
            imu_addr: int = _DEFAULT_IMU_ADDR) -> None:
    """
    Initialise I2C bus, initialise the IMU, and start the background reader thread.

    Args:
        i2c_bus:  Raspberry Pi I2C bus number (almost always 1).
        imu_addr: ICM-20948 I2C address (0x68 AD0-low, 0x69 AD0-high).
    """
    global _bus, _imu_addr, _thread, _running

    _imu_addr = imu_addr
    _bus = smbus2.SMBus(i2c_bus)

    _init_imu()

    _running = True
    _thread = threading.Thread(target=_reader, daemon=True)
    _thread.start()


def disconnect() -> None:
    """Stop the background thread and release I2C bus."""
    global _running, _bus
    _running = False
    if _thread is not None:
        _thread.join(timeout=1.0)
    if _bus is not None:
        _bus.close()


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
    """Return the averaged LIDAR distance in metres, or None if unavailable."""
    with _lock:
        return _latest_lidar


def is_imu_ok() -> bool:
    with _lock:
        return not _imu_error and _latest_imu is not None


def is_lidar_ok() -> bool:
    with _lock:
        return not _lidar_error and _latest_lidar is not None


# ── IMU initialisation ────────────────────────────────────────────────────────
def _icm_bank(bank: int) -> None:
    _bus.write_byte_data(_imu_addr, _REG_BANK_SEL, bank << 4)


def _init_imu() -> None:
    global _imu_error
    try:
        _icm_bank(0)
        _bus.write_byte_data(_imu_addr, _PWR_MGMT_1, 0x01)   # wake, auto clock
        time.sleep(0.1)
        _bus.write_byte_data(_imu_addr, _INT_PIN_CFG, 0x02)  # I2C bypass for mag
        time.sleep(0.02)
        _bus.write_byte_data(_MAG_ADDR, _MAG_CNTL2, 0x08)    # 100 Hz continuous
        time.sleep(0.02)
    except OSError:
        with _lock:
            _imu_error = True


# ── Sensor reads ──────────────────────────────────────────────────────────────
def _read_imu_sample() -> dict | None:
    global _last_mx, _last_my, _last_mz
    try:
        _icm_bank(0)
        # Accel (6 bytes) + Gyro (6 bytes) are contiguous from 0x2D
        raw = _bus.read_i2c_block_data(_imu_addr, _ACCEL_XOUT_H, 12)
        ax = struct.unpack('>h', bytes(raw[0:2]))[0]  / _ACCEL_SCALE
        ay = struct.unpack('>h', bytes(raw[2:4]))[0]  / _ACCEL_SCALE
        az = struct.unpack('>h', bytes(raw[4:6]))[0]  / _ACCEL_SCALE
        gx = struct.unpack('>h', bytes(raw[6:8]))[0]  / _GYRO_SCALE
        gy = struct.unpack('>h', bytes(raw[8:10]))[0] / _GYRO_SCALE
        gz = struct.unpack('>h', bytes(raw[10:12]))[0]/ _GYRO_SCALE

        # Magnetometer — only update when data-ready
        st1 = _bus.read_byte_data(_MAG_ADDR, _MAG_ST1)
        if st1 & 0x01:
            mb = _bus.read_i2c_block_data(_MAG_ADDR, _MAG_HXL, 8)
            _last_mx = struct.unpack('<h', bytes(mb[0:2]))[0]  # AK09916 little-endian
            _last_my = struct.unpack('<h', bytes(mb[2:4]))[0]
            _last_mz = struct.unpack('<h', bytes(mb[4:6]))[0]
            # reading mb[7] (ST2) releases the latch — already consumed above

        return {"ax": ax, "ay": ay, "az": az,
                "gx": gx, "gy": gy, "gz": gz,
                "mx": _last_mx, "my": _last_my, "mz": _last_mz}
    except OSError:
        return None


def _read_one_lidar(addr: int) -> float | None:
    """Trigger and read one LIDAR-Lite v4. Returns distance in metres or None."""
    try:
        _bus.write_byte_data(addr, 0x00, 0x04)   # trigger measurement
        time.sleep(0.02)
        high = _bus.read_byte_data(addr, 0x0f)
        low  = _bus.read_byte_data(addr, 0x10)
        cm = (high << 8) | low
        return cm / 100.0
    except OSError:
        return None


def _read_lidar_averaged() -> float | None:
    d1 = _read_one_lidar(_LIDAR_ADDR_1)
    d2 = _read_one_lidar(_LIDAR_ADDR_2)

    if d1 is None and d2 is None:
        return None
    if d1 is None:
        return d2
    if d2 is None:
        return d1

    # Both valid — average if within tolerance, else take the closer reading
    if abs(d1 - d2) * 100 <= _LIDAR_TOLERANCE_CM:
        return (d1 + d2) / 2.0
    return min(d1, d2)


# ── Background reader thread ──────────────────────────────────────────────────
def _reader() -> None:
    global _latest_imu, _latest_lidar, _imu_error, _lidar_error

    while _running:
        imu_sample = _read_imu_sample()
        with _lock:
            if imu_sample is not None:
                _latest_imu = imu_sample
                _imu_error  = False
            else:
                _imu_error = True

        lidar_dist = _read_lidar_averaged()
        with _lock:
            if lidar_dist is not None:
                _latest_lidar = lidar_dist
                _lidar_error  = False
            else:
                _lidar_error = True

        time.sleep(0.05)   # ~20 Hz poll rate

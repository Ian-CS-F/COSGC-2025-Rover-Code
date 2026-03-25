"""
Test Protocol: Sensors (Lidar + IMU)
Tests the Garmin LIDAR-Lite v4 LED and ICM-20948 IMU exactly as used in main.py.
Requires I2C connections to both devices. No Arduino needed.
"""

import struct
import time
import smbus2  # type: ignore

# ── Constants from main.py ────────────────────────────────────────────────────
LIDAR_ADDR   = 0x62
ICM_ADDR     = 0x68
MAG_ADDR     = 0x0C
REG_BANK_SEL = 0x7F
_PWR_MGMT_1  = 0x06
_INT_PIN_CFG = 0x0F
_ACCEL_XOUT_H = 0x2D
_MAG_CNTL2   = 0x31
I2C_BUS      = 1

# ── Helpers copied exactly from main.py ───────────────────────────────────────
def _icm_bank(bus, bank):
    bus.write_byte_data(ICM_ADDR, REG_BANK_SEL, bank << 4)

def read_lidar(bus):
    """Trigger a measurement and return distance in metres."""
    bus.write_byte_data(LIDAR_ADDR, 0x00, 0x04)
    time.sleep(0.02)
    high = bus.read_byte_data(LIDAR_ADDR, 0x0f)
    low  = bus.read_byte_data(LIDAR_ADDR, 0x10)
    return ((high << 8) | low) / 100.0

def read_accel_z(bus):
    _icm_bank(bus, 0)
    raw = bus.read_i2c_block_data(ICM_ADDR, _ACCEL_XOUT_H, 6)
    return struct.unpack(">h", bytes(raw[4:6]))[0] / 16384.0

def init_imu(bus):
    _icm_bank(bus, 0)
    bus.write_byte_data(ICM_ADDR, _PWR_MGMT_1,  0x01)
    time.sleep(0.1)
    bus.write_byte_data(ICM_ADDR, _INT_PIN_CFG, 0x02)
    time.sleep(0.02)
    bus.write_byte_data(MAG_ADDR, _MAG_CNTL2,   0x08)
    time.sleep(0.02)

# ── Tests ─────────────────────────────────────────────────────────────────────
def test_lidar_returns_distance(bus):
    """Lidar triggers a measurement and returns a positive distance in metres."""
    print("TEST: Lidar returns a distance reading...")
    dist = read_lidar(bus)
    passed = isinstance(dist, float) and dist > 0.0
    print(f"  Distance: {dist:.3f} m  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_lidar_range_plausible(bus):
    """Lidar distance is within a plausible indoor range (0.02 – 40 m)."""
    print("TEST: Lidar distance is in a plausible range...")
    dist = read_lidar(bus)
    passed = 0.02 <= dist <= 40.0
    print(f"  Distance: {dist:.3f} m  →  {'PASS' if passed else 'FAIL (out of 0.02–40 m range)'}")
    return passed

def test_lidar_consistent(bus, n=5):
    """Five consecutive readings should be within 0.5 m of each other (no wild jumps)."""
    print(f"TEST: Lidar gives consistent readings across {n} samples...")
    readings = [read_lidar(bus) for _ in range(n)]
    spread = max(readings) - min(readings)
    passed = spread < 0.5
    print(f"  Readings: {[f'{r:.3f}' for r in readings]}  spread={spread:.3f} m  →  {'PASS' if passed else 'FAIL'}")
    return passed

def test_imu_accel_z_flat(bus):
    """On a flat surface, IMU Z-axis should read close to +1 g (gravity)."""
    print("TEST: IMU Z-axis reads ~+1 g on flat surface...")
    az = read_accel_z(bus)
    passed = 0.7 <= az <= 1.3
    print(f"  az = {az:.3f} g  →  {'PASS' if passed else 'FAIL (expected 0.7–1.3 g on flat surface)'}")
    return passed

def test_imu_not_inverted(bus):
    """Rover should NOT appear inverted (az > −0.5 g means upright)."""
    print("TEST: IMU confirms rover is upright (not flipped)...")
    az = read_accel_z(bus)
    passed = az > -0.5
    print(f"  az = {az:.3f} g  →  {'PASS' if passed else 'FAIL (rover appears inverted)'}")
    return passed

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    bus = smbus2.SMBus(I2C_BUS)
    init_imu(bus)

    lidar_tests = [test_lidar_returns_distance, test_lidar_range_plausible, test_lidar_consistent]
    imu_tests   = [test_imu_accel_z_flat, test_imu_not_inverted]

    results = []
    print("\n=== LIDAR ===")
    for t in lidar_tests:
        results.append((t.__name__, t(bus)))

    print("\n=== IMU ===")
    for t in imu_tests:
        results.append((t.__name__, t(bus)))

    bus.close()

    print("\n--- Sensor Test Results ---")
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")

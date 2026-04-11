/*
 * imu.ino — ICM-20948 gyroscope driver (standalone, Arduino Mega)
 * ================================================================
 * Provides heading tracking using the ICM-20948 gyroscope Z axis.
 * No magnetometer used — purely gyro integration, so heading drifts
 * slowly over time but is accurate enough for short drive segments.
 *
 * Wiring (Arduino Mega):
 *   ICM-20948 SDA → pin 20
 *   ICM-20948 SCL → pin 21
 *   VCC           → 3.3V
 *   GND           → GND
 *   AD0           → 3.3V  (I2C address 0x69)
 *
 * Public API (called from rover_standalone.ino):
 *   bool  imuBegin()          — initialise; returns false if not found
 *   void  imuResetHeading()   — zero the integrated heading
 *   float imuHeadingDeg()     — degrees since last reset (+ = right, - = left)
 *   void  imuUpdate()         — call as often as possible to integrate gyro
 */

#include <Wire.h>
#include "config.h"

// ── ICM-20948 register map (minimal — gyro only) ──────────────────────────────
#define ICM_ADDR       0x69

#define REG_BANK_SEL   0x7F

// Bank 0
#define B0_WHO_AM_I    0x00   // expected: 0xEA
#define B0_PWR_MGMT_1  0x06
#define B0_GYRO_ZOUT_H 0x33
#define B0_GYRO_ZOUT_L 0x34

// Bank 2
#define B2_GYRO_CONFIG_1    0x01   // DLPF + range
#define B2_GYRO_SMPLRT_DIV  0x00

// Gyro sensitivity at ±250 dps (GYRO_FS_SEL = 0b00)
#define GYRO_SCALE_DPS  131.0f

// ── Internal state ────────────────────────────────────────────────────────────
static float    _headingDeg    = 0.0f;
static uint32_t _lastUpdateUs  = 0;
static bool     _imuReady      = false;

// ── Register helpers ──────────────────────────────────────────────────────────
static void _selectBank(uint8_t bank) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(REG_BANK_SEL);
    Wire.write((bank & 0x03) << 4);
    Wire.endTransmission();
}

static void _writeReg(uint8_t bank, uint8_t reg, uint8_t val) {
    _selectBank(bank);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t _readReg(uint8_t bank, uint8_t reg) {
    _selectBank(bank);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static int16_t _readGyroZ() {
    _selectBank(0);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(B0_GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
}

// ── Public API ────────────────────────────────────────────────────────────────
bool imuBegin() {
    Wire.begin();
    delay(10);

    // Wake device, select best clock source
    _writeReg(0, B0_PWR_MGMT_1, 0x01);
    delay(50);

    uint8_t who = _readReg(0, B0_WHO_AM_I);
    if (who != 0xEA) {
        Serial.print("IMU: WHO_AM_I = 0x");
        Serial.print(who, HEX);
        Serial.println(" (expected 0xEA) — check wiring");
        return false;
    }

    // Gyro ±250 dps, enable DLPF (reduces noise)
    _writeReg(2, B2_GYRO_CONFIG_1, 0x01);
    // Sample rate divider = 0 → max rate
    _writeReg(2, B2_GYRO_SMPLRT_DIV, 0x00);

    _lastUpdateUs = micros();
    _headingDeg   = 0.0f;
    _imuReady     = true;

    Serial.println("IMU: ICM-20948 found and configured.");
    return true;
}

// Zero the heading reference — call at the start of each drive segment.
void imuResetHeading() {
    _headingDeg  = 0.0f;
    _lastUpdateUs = micros();
}

// Integrate gyro Z into heading — call as often as possible (every ~10ms).
void imuUpdate() {
    if (!_imuReady) return;
    uint32_t now = micros();
    float dt = (now - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = now;

    float gyroZ = _readGyroZ() / GYRO_SCALE_DPS;  // dps
    _headingDeg += gyroZ * dt;                      // integrate → degrees
}

// Heading in degrees since last reset.  + = turned right, - = turned left.
float imuHeadingDeg() {
    return _headingDeg;
}

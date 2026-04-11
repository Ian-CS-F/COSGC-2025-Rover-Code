/*
 * lidar.ino — LIDAR-Lite v4 driver (standalone, Arduino Mega)
 * ============================================================
 * Drives up to two LIDAR-Lite v4 units over I2C (pins 20/21).
 * Shares the bus with the IMU — Wire.begin() is called by imu.ino.
 *
 * Address trust order:
 *   LIDAR_ADDR_PRIMARY   0x5b  — more trusted unit, used first
 *   LIDAR_ADDR_SECONDARY 0x62  — backup unit
 *
 * lidarForwardCm() returns the PRIMARY reading if available,
 * falling back to secondary if primary fails, then heightmap if neither found.
 *
 * Wiring:
 *   SDA → pin 20, SCL → pin 21 (shared with IMU, level-shifted to 3.3V)
 *   VCC → 3.3V, GND → GND
 *
 * Public API:
 *   bool  lidarBegin()          — probe both units, returns true if any found
 *   float lidarForwardCm()      — best distance reading in cm (primary preferred)
 */

// ── LIDAR-Lite v4 registers ───────────────────────────────────────────────────
#define LIDAR_REG_CMD        0x00
#define LIDAR_REG_STATUS     0x01   // bit 0: 1=busy, 0=ready
#define LIDAR_REG_DIST_HIGH  0x0F
#define LIDAR_REG_DIST_LOW   0x10

#define LIDAR_CMD_MEASURE    0x04
#define LIDAR_TIMEOUT_MS     50
#define LIDAR_NO_READING     999.0f

// ── Addresses — 0x5b is more trusted ─────────────────────────────────────────
#define LIDAR_ADDR_PRIMARY   0x5b
#define LIDAR_ADDR_SECONDARY 0x62

static bool _lidarPrimaryOk   = false;
static bool _lidarSecondaryOk = false;

// ── Internal helpers ──────────────────────────────────────────────────────────
static bool _lidarProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

static float _lidarMeasure(uint8_t addr) {
    // Trigger
    Wire.beginTransmission(addr);
    Wire.write(LIDAR_REG_CMD);
    Wire.write(LIDAR_CMD_MEASURE);
    if (Wire.endTransmission() != 0) return LIDAR_NO_READING;

    // Wait until ready
    unsigned long deadline = millis() + LIDAR_TIMEOUT_MS;
    while (millis() < deadline) {
        Wire.beginTransmission(addr);
        Wire.write(LIDAR_REG_STATUS);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)addr, (uint8_t)1);
        if (Wire.available() && !(Wire.read() & 0x01)) break;
        delay(1);
    }
    if (millis() >= deadline) return LIDAR_NO_READING;

    // Read distance
    Wire.beginTransmission(addr);
    Wire.write(LIDAR_REG_DIST_HIGH);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr, (uint8_t)2);
    if (Wire.available() < 2) return LIDAR_NO_READING;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (float)((hi << 8) | lo);
}

// ── Public API ────────────────────────────────────────────────────────────────
bool lidarBegin() {
    // Wire.begin() already called by imu.ino
    _lidarPrimaryOk   = _lidarProbe(LIDAR_ADDR_PRIMARY);
    _lidarSecondaryOk = _lidarProbe(LIDAR_ADDR_SECONDARY);

    Serial.print("LIDAR 0x5b (primary):   ");
    Serial.println(_lidarPrimaryOk   ? "found" : "NOT found");
    Serial.print("LIDAR 0x62 (secondary): ");
    Serial.println(_lidarSecondaryOk ? "found" : "NOT found");

    return _lidarPrimaryOk || _lidarSecondaryOk;
}

// Returns primary (0x5b) reading if available, secondary (0x62) as fallback.
// Returns LIDAR_NO_READING (999) if neither unit responds.
float lidarForwardCm() {
    if (_lidarPrimaryOk) {
        float d = _lidarMeasure(LIDAR_ADDR_PRIMARY);
        if (d < LIDAR_NO_READING) return d;
    }
    if (_lidarSecondaryOk) {
        float d = _lidarMeasure(LIDAR_ADDR_SECONDARY);
        if (d < LIDAR_NO_READING) return d;
    }
    return LIDAR_NO_READING;
}

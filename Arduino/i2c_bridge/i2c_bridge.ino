/*
 * i2c_bridge.ino
 * Reads the ICM-20948 IMU and Garmin LIDAR-Lite v4 over I2C, then
 * streams formatted sensor data over USB serial at 9600 baud.
 *
 * This lets a laptop run rover_main.py without a Raspberry Pi — the
 * laptop reads this serial stream instead of talking to I2C directly.
 *
 * Wiring (Uno / Mega):
 *   SDA → A4 (Uno) or 20 (Mega)
 *   SCL → A5 (Uno) or 21 (Mega)
 *   ICM-20948 AD0 → 3.3 V  (sets I2C address to 0x69)
 *
 * Output lines (one pair every 100 ms):
 *   IMU:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>
 *   LIDAR:<dist_m>
 *   ERROR:IMU    ← if I2C read fails
 *   ERROR:LIDAR  ← if I2C read fails
 */

#include <Wire.h>

// ── Addresses ─────────────────────────────────────────────────────────────────
#define ICM_ADDR   0x69   // AD0 pulled HIGH
#define MAG_ADDR   0x0C   // AK09916 magnetometer (exposed via bypass mode)
#define LIDAR_ADDR 0x62

// ── ICM-20948 register map (bank 0) ───────────────────────────────────────────
#define REG_BANK_SEL   0x7F
#define PWR_MGMT_1     0x06
#define INT_PIN_CFG    0x0F
#define ACCEL_XOUT_H   0x2D   // 6 bytes: ax_h,ax_l,ay_h,ay_l,az_h,az_l
#define GYRO_XOUT_H    0x33   // 6 bytes: gx_h,gx_l,gy_h,gy_l,gz_h,gz_l

// ── AK09916 magnetometer registers ────────────────────────────────────────────
#define MAG_CNTL2  0x31   // write 0x08 → continuous 100 Hz
#define MAG_ST1    0x10   // bit 0 = data ready
#define MAG_HXL    0x11   // 8 bytes: xl,xh,yl,yh,zl,zh,tmps,st2

// ── Scale factors ─────────────────────────────────────────────────────────────
#define ACCEL_SCALE 16384.0f   // ±2 g  → LSB/g
#define GYRO_SCALE    131.0f   // ±250 dps → LSB/dps

// ── Timing ────────────────────────────────────────────────────────────────────
#define SEND_INTERVAL_MS 100   // send one IMU + LIDAR pair every 100 ms
#define LIDAR_MEAS_MS     20   // LIDAR measurement time per datasheet

// ── I2C helpers ───────────────────────────────────────────────────────────────

// Select ICM register bank (0–3)
void icmBank(uint8_t bank) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(REG_BANK_SEL);
    Wire.write(bank << 4);
    Wire.endTransmission();
}

// Write one byte to an ICM register. Returns true on success.
bool icmWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Read n bytes from ICM starting at reg into buf. Returns true on success.
bool icmRead(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    uint8_t got = Wire.requestFrom((uint8_t)ICM_ADDR, n);
    if (got < n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

// Read one byte from any device. Returns true on success.
bool i2cReadByte(uint8_t addr, uint8_t reg, uint8_t &out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) < 1) return false;
    out = Wire.read();
    return true;
}

// ── IMU initialisation ────────────────────────────────────────────────────────
bool imuReady = false;

bool initIMU() {
    icmBank(0);

    // Wake up and select auto clock
    if (!icmWrite(PWR_MGMT_1, 0x01)) return false;
    delay(100);

    // Enable I2C bypass so AK09916 appears directly on the bus
    if (!icmWrite(INT_PIN_CFG, 0x02)) return false;
    delay(20);

    // Configure AK09916: continuous measurement at 100 Hz
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_CNTL2);
    Wire.write(0x08);
    if (Wire.endTransmission() != 0) return false;
    delay(20);

    return true;
}

// ── IMU read ──────────────────────────────────────────────────────────────────
struct IMUSample {
    float ax, ay, az;
    float gx, gy, gz;
    int16_t mx, my, mz;
};

bool readIMU(IMUSample &s) {
    icmBank(0);

    // Accel (6 bytes at 0x2D) and Gyro (6 bytes at 0x33) are contiguous —
    // read all 12 in one transaction.
    uint8_t buf[12];
    if (!icmRead(ACCEL_XOUT_H, buf, 12)) return false;

    s.ax = (int16_t)((buf[0]  << 8) | buf[1])  / ACCEL_SCALE;
    s.ay = (int16_t)((buf[2]  << 8) | buf[3])  / ACCEL_SCALE;
    s.az = (int16_t)((buf[4]  << 8) | buf[5])  / ACCEL_SCALE;
    s.gx = (int16_t)((buf[6]  << 8) | buf[7])  / GYRO_SCALE;
    s.gy = (int16_t)((buf[8]  << 8) | buf[9])  / GYRO_SCALE;
    s.gz = (int16_t)((buf[10] << 8) | buf[11]) / GYRO_SCALE;

    // Magnetometer — only read when data-ready bit is set
    uint8_t st1 = 0;
    i2cReadByte(MAG_ADDR, MAG_ST1, st1);
    if (st1 & 0x01) {
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(MAG_HXL);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)8) < 8) return false;
        uint8_t mb[8];
        for (int i = 0; i < 8; i++) mb[i] = Wire.read();
        // AK09916 is little-endian
        s.mx = (int16_t)((mb[1] << 8) | mb[0]);
        s.my = (int16_t)((mb[3] << 8) | mb[2]);
        s.mz = (int16_t)((mb[5] << 8) | mb[4]);
        // Reading mb[7] (ST2) automatically releases the data latch
    }
    // If DRDY not set, keep previous mag values (they're initialised to 0)

    return true;
}

// ── LIDAR read ────────────────────────────────────────────────────────────────
// Returns distance in metres, or -1.0 on I2C error.
float readLidar() {
    // Trigger a measurement
    Wire.beginTransmission(LIDAR_ADDR);
    Wire.write(0x00);
    Wire.write(0x04);
    if (Wire.endTransmission() != 0) return -1.0f;

    delay(LIDAR_MEAS_MS);

    // Read high byte (register 0x0f)
    uint8_t high, low;
    if (!i2cReadByte(LIDAR_ADDR, 0x0f, high)) return -1.0f;
    if (!i2cReadByte(LIDAR_ADDR, 0x10, low))  return -1.0f;

    return ((uint16_t)(high << 8) | low) / 100.0f;
}

// ── Arduino setup / loop ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);   // 400 kHz fast mode

    imuReady = initIMU();
    if (!imuReady) {
        Serial.println("ERROR:IMU_INIT");
    }
}

unsigned long lastSend = 0;
IMUSample lastIMU = {0};   // keep last good mag reading across loops

void loop() {
    if (millis() - lastSend < SEND_INTERVAL_MS) return;
    lastSend = millis();

    // ── IMU ──────────────────────────────────────────────────────────────────
    if (imuReady) {
        IMUSample s = lastIMU;   // preserve last mag if DRDY not set
        if (readIMU(s)) {
            lastIMU = s;
            Serial.print("IMU:");
            Serial.print(s.ax, 4); Serial.print(',');
            Serial.print(s.ay, 4); Serial.print(',');
            Serial.print(s.az, 4); Serial.print(',');
            Serial.print(s.gx, 4); Serial.print(',');
            Serial.print(s.gy, 4); Serial.print(',');
            Serial.print(s.gz, 4); Serial.print(',');
            Serial.print(s.mx);    Serial.print(',');
            Serial.print(s.my);    Serial.print(',');
            Serial.println(s.mz);
        } else {
            Serial.println("ERROR:IMU");
        }
    } else {
        Serial.println("ERROR:IMU");
    }

    // ── LIDAR ─────────────────────────────────────────────────────────────────
    float dist = readLidar();
    if (dist >= 0.0f) {
        Serial.print("LIDAR:");
        Serial.println(dist, 4);
    } else {
        Serial.println("ERROR:LIDAR");
    }
}

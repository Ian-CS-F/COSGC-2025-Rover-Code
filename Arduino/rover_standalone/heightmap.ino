/*
 * heightmap.ino — ultrasonic tilt-sweep heightmap (standalone, Arduino Mega)
 * ===========================================================================
 * Sweeps the tilt servo through a range of angles, reads the HC-SR04
 * ultrasonic at each position, and converts (angle, distance) pairs into
 * (horizontal distance from rover, height above ground) using basic
 * trigonometry — the same principle as the LIDAR heightmap on the Pi.
 *
 * Geometry:
 *   Servo 90°  = horizontal (looking straight ahead)
 *   Servo >90° = tilted downward
 *   At angle θ (servo degrees):
 *     α = (θ − 90°) in radians  — deviation below horizontal
 *     horizontal distance from sensor = d · cos(α)
 *     height of object above ground   = SENSOR_HEIGHT_CM − d · sin(α)
 *
 * Heightmap cells store horizontal distance and object height.
 * Obstacle detection: object height in [OBSTACLE_MIN_CM, OBSTACLE_MAX_CM]
 *                     and within OBSTACLE_THRESHOLD_CM horizontally
 * Cliff detection:    object height below −CLIFF_DROP_CM  (floor missing)
 *
 * Tune:
 *   SENSOR_HEIGHT_CM   — measure: height of ultrasonic above floor (cm)
 *   SCAN_TILT_MIN      — lowest servo angle in sweep (most downward)
 *   SCAN_TILT_MAX      — highest servo angle in sweep (most upward / forward)
 *   SCAN_STEPS         — number of samples across the sweep
 *   OBSTACLE_MIN_CM    — minimum object height to count as obstacle
 *   OBSTACLE_MAX_CM    — maximum object height to count as obstacle
 *   CLIFF_DROP_CM      — how far below ground before classing as cliff
 *
 * Public API:
 *   void  hmScan()              — perform sweep and populate heightmap
 *   bool  hmObstacleAhead()     — true if obstacle blocks path
 *   bool  hmCliffAhead()        — true if cliff/drop detected
 *   void  hmPrint()             — dump heightmap to Serial for debugging
 */

#include <math.h>

// ── Heightmap tuning ──────────────────────────────────────────────────────────
#define SENSOR_HEIGHT_CM   20.0f   // height of ultrasonic above ground — MEASURE THIS

#define SCAN_TILT_MIN      90      // servo degrees — upward extreme (tall obstacle detection)
#define SCAN_TILT_MAX      140     // servo degrees — downward extreme (cliff detection)
                                   // 90° = horizontal, >90° = below horizontal
#define SCAN_STEPS         10      // samples across sweep
#define SCAN_SETTLE_MS     250     // ms to wait after each servo move

#define OBSTACLE_MIN_CM     5.0f   // objects below this height are floor — ignore
#define OBSTACLE_MAX_CM    60.0f   // objects above this are overhead — ignore
#define CLIFF_DROP_CM      10.0f   // object height this far below ground = cliff

// ── Heightmap storage ─────────────────────────────────────────────────────────
struct HmCell {
    float hDist;    // horizontal distance from rover (cm)
    float height;   // height of detected surface above ground (cm)
    bool  valid;    // false if ultrasonic returned no echo
};

static HmCell _hm[SCAN_STEPS];
static int    _hmCount = 0;

// ── Internal helpers ──────────────────────────────────────────────────────────
// readUltrasonic() and setTilt() are defined in rover_standalone.ino and
// are accessible here because all .ino files in the same folder compile together.

static void _hmSample(int step, int tiltDeg) {
    setTilt(tiltDeg);
    delay(SCAN_SETTLE_MS);
    float dist = readUltrasonic();

    float alphaRad = (tiltDeg - 90) * (float)M_PI / 180.0f;  // + = below horizontal
    _hm[step].valid = (dist < 500.0f);
    if (_hm[step].valid) {
        _hm[step].hDist  = dist * cos(alphaRad);
        _hm[step].height = SENSOR_HEIGHT_CM - dist * sin(alphaRad);
    } else {
        // No echo — treat as very far away, height unknown
        _hm[step].hDist  = 999.0f;
        _hm[step].height = 999.0f;
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

// Sweep the tilt servo and build the heightmap.
// Call this before hmObstacleAhead() or hmCliffAhead().
void hmScan() {
    _hmCount = SCAN_STEPS;
    int range = SCAN_TILT_MAX - SCAN_TILT_MIN;
    for (int i = 0; i < SCAN_STEPS; i++) {
        int angle = SCAN_TILT_MIN + (range * i) / (SCAN_STEPS - 1);
        _hmSample(i, angle);
    }
    // Return servo to forward position after scan
    setTilt(90);
}

// True if any heightmap cell shows an object in the obstacle height band
// within the forward obstacle threshold distance.
// Uses OBSTACLE_THRESHOLD_CM from rover_standalone.ino (compiled together).
bool hmObstacleAhead() {
    for (int i = 0; i < _hmCount; i++) {
        if (!_hm[i].valid) continue;
        if (_hm[i].hDist  < OBSTACLE_THRESHOLD_CM &&
            _hm[i].height > OBSTACLE_MIN_CM        &&
            _hm[i].height < OBSTACLE_MAX_CM) {
            return true;
        }
    }
    return false;
}

// True if any heightmap cell indicates a cliff (floor missing ahead).
bool hmCliffAhead() {
    int downwardSamples = 0;
    int missingFloor    = 0;

    for (int i = 0; i < _hmCount; i++) {
        // Reconstruct the tilt angle for this sample index
        int range    = SCAN_TILT_MAX - SCAN_TILT_MIN;
        int tiltDeg  = SCAN_TILT_MIN + (range * i) / (SCAN_STEPS - 1);
        bool isDown  = (tiltDeg > 92);  // small margin around horizontal

        if (isDown) downwardSamples++;   // count every downward sample once

        if (_hm[i].valid) {
            // Object detected far below ground = cliff/drop
            if (_hm[i].height < -CLIFF_DROP_CM) return true;
        } else if (isDown) {
            // No echo on a downward sample = floor missing = cliff
            missingFloor++;
        }
    }

    // If majority of downward samples have no floor echo, call it a cliff
    if (downwardSamples > 0 && missingFloor > downwardSamples / 2) return true;
    return false;
}

// Print heightmap to Serial for debugging / calibration.
void hmPrint() {
    Serial.println("--- HEIGHTMAP ---");
    for (int i = 0; i < _hmCount; i++) {
        Serial.print("  cell ");
        Serial.print(i);
        if (_hm[i].valid) {
            Serial.print(": hDist=");
            Serial.print(_hm[i].hDist, 1);
            Serial.print("cm  height=");
            Serial.print(_hm[i].height, 1);
            Serial.println("cm");
        } else {
            Serial.println(": no echo");
        }
    }
    Serial.println("-----------------");
}

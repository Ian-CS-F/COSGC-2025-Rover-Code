/*
 * navigation.ino — zone-based reactive navigation (standalone, Arduino Mega)
 * ===========================================================================
 * After each heightmap sweep, scores three horizontal zones (LEFT, CENTER,
 * RIGHT) and returns the best direction to travel.
 *
 * Scan sequence (rover returns to original heading after each scan):
 *   1. Read CENTER — forward horizontal ultrasonic read
 *   2. Turn left NAV_SCAN_ANGLE, read LEFT, turn back
 *   3. Turn right NAV_SCAN_ANGLE, read RIGHT, turn back
 *
 * Scoring per zone:
 *   base score  = distance reading (further = better)
 *   cliff penalty  = -1000  (zone treated as impassable)
 *   blocked penalty= -500   (reading < NAV_MIN_CLEAR_CM)
 *   open bonus     = +200   (no echo = open space)
 *   center bias    = +20    (prefer straight when equal)
 *
 * Returns: NAV_CENTER, NAV_LEFT, NAV_RIGHT, or NAV_BACK (all blocked)
 *
 * Memory: 3 × ZoneScore (9 bytes each) = 27 bytes total. No heap allocation.
 *
 * Integration: call chooseDirection() from loop() in rover_standalone.ino.
 *   All motor, encoder, IMU, and heightmap code is unchanged.
 *   readUltrasonic(), setTilt(), turnLeft(), turnRight() must be defined
 *   in rover_standalone.ino (they are, compiled together automatically).
 *
 * Tuning:
 *   NAV_SCAN_ANGLE    degrees to turn left/right for zone reads
 *   NAV_SCAN_TILT     tilt angle for horizontal reads (90 = forward level)
 *   NAV_TILT_SETTLE   ms to wait after tilt before reading
 *   NAV_TURN_SETTLE   ms to wait after tank turn before reading
 *   NAV_MIN_CLEAR_CM  distance below which a zone is considered blocked
 *   NAV_CENTER_BIAS   score bonus for center zone (prefer straight)
 */

// ── Tuning ────────────────────────────────────────────────────────────────────
#define NAV_SCAN_ANGLE    30      // degrees to turn for left/right zone reads
#define NAV_SCAN_TILT     90      // servo angle for horizontal read (level)
#define NAV_TILT_SETTLE   200     // ms after tilt move before reading
#define NAV_TURN_SETTLE   150     // ms after tank turn before reading
#define NAV_MIN_CLEAR_CM  15.0f   // reading below this = zone blocked
#define NAV_CENTER_BIAS   20.0f   // score bonus added to center zone

// ── Direction constants ───────────────────────────────────────────────────────
#define NAV_CENTER  0
#define NAV_LEFT    1
#define NAV_RIGHT   2
#define NAV_BACK    3   // all zones blocked — caller should turn 180°

// ── Zone state ────────────────────────────────────────────────────────────────
struct ZoneScore {
    float distance;   // raw ultrasonic reading (cm)
    bool  cliff;      // cliff flagged in this zone
    float score;      // computed score (higher = better)
};

static ZoneScore _zones[3];   // [NAV_CENTER, NAV_LEFT, NAV_RIGHT]

// ── Internal helpers ──────────────────────────────────────────────────────────

// Point tilt servo level and take one distance reading.
// Uses LIDAR if available (faster, more accurate), ultrasonic otherwise.
// lidarAvailable is defined in rover_standalone.ino (compiled together).
static float _quickRead() {
    if (lidarAvailable) {
        return lidarForwardCm();   // LIDAR: no servo move needed, instant read
    }
    setTilt(NAV_SCAN_TILT);
    delay(NAV_TILT_SETTLE);
    return readUltrasonic();
}

// Compute a score for a zone given its distance and cliff status.
// Higher score = safer / more desirable direction.
static float _score(float dist, bool cliff) {
    if (cliff)              return -1000.0f;
    if (dist < NAV_MIN_CLEAR_CM) return -500.0f;   // blocked
    if (dist >= 998.0f)    return  200.0f;          // open — no echo
    return dist;                                     // further = better
}

// ── Public API ────────────────────────────────────────────────────────────────

/*
 * chooseDirection()
 *
 * Scans left, center, and right zones using quick horizontal ultrasonic reads.
 * Uses cliff flags from the last hmScan() call (call hmScan() before this).
 * The rover returns to its original heading after the scan.
 *
 * Returns NAV_CENTER, NAV_LEFT, NAV_RIGHT, or NAV_BACK.
 */
int chooseDirection() {
    // ── 1. Center scan ────────────────────────────────────────────────────────
    _zones[NAV_CENTER].distance = _quickRead();
    _zones[NAV_CENTER].cliff    = hmCliffAhead();   // uses last hmScan() result

    // ── 2. Left scan — turn left, read, return ────────────────────────────────
    turnLeft(NAV_SCAN_ANGLE);
    delay(NAV_TURN_SETTLE);
    _zones[NAV_LEFT].distance = _quickRead();
    _zones[NAV_LEFT].cliff    = false;  // no cliff sweep for side zones
    turnRight(NAV_SCAN_ANGLE);          // return to original heading
    delay(NAV_TURN_SETTLE);

    // ── 3. Right scan — turn right, read, return ──────────────────────────────
    turnRight(NAV_SCAN_ANGLE);
    delay(NAV_TURN_SETTLE);
    _zones[NAV_RIGHT].distance = _quickRead();
    _zones[NAV_RIGHT].cliff    = false;
    turnLeft(NAV_SCAN_ANGLE);           // return to original heading
    delay(NAV_TURN_SETTLE);

    // ── 4. Score each zone ────────────────────────────────────────────────────
    _zones[NAV_CENTER].score = _score(_zones[NAV_CENTER].distance,
                                      _zones[NAV_CENTER].cliff) + NAV_CENTER_BIAS;
    _zones[NAV_LEFT  ].score = _score(_zones[NAV_LEFT  ].distance,
                                      _zones[NAV_LEFT  ].cliff);
    _zones[NAV_RIGHT ].score = _score(_zones[NAV_RIGHT ].distance,
                                      _zones[NAV_RIGHT ].cliff);

    // ── 5. Serial debug ───────────────────────────────────────────────────────
    Serial.print("NAV  L:");
    Serial.print(_zones[NAV_LEFT  ].distance, 0); Serial.print("cm(");
    Serial.print(_zones[NAV_LEFT  ].score, 0);    Serial.print(")  C:");
    Serial.print(_zones[NAV_CENTER].distance, 0); Serial.print("cm(");
    Serial.print(_zones[NAV_CENTER].score, 0);    Serial.print(")  R:");
    Serial.print(_zones[NAV_RIGHT ].distance, 0); Serial.print("cm(");
    Serial.print(_zones[NAV_RIGHT ].score, 0);    Serial.print(")  → ");

    // ── 6. Pick best zone ─────────────────────────────────────────────────────
    int   best      = NAV_CENTER;
    float bestScore = _zones[NAV_CENTER].score;

    if (_zones[NAV_LEFT ].score > bestScore) {
        best      = NAV_LEFT;
        bestScore = _zones[NAV_LEFT].score;
    }
    if (_zones[NAV_RIGHT].score > bestScore) {
        best      = NAV_RIGHT;
        bestScore = _zones[NAV_RIGHT].score;
    }

    // All zones blocked or cliff — signal caller to reverse
    if (bestScore < 0.0f) {
        Serial.println("BACK");
        return NAV_BACK;
    }

    const char* names[] = {"CENTER", "LEFT", "RIGHT"};
    Serial.println(names[best]);
    return best;
}

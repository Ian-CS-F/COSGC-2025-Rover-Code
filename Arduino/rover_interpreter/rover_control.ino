/*
 * rover_control.ino
 * =================
 * Higher-level motion control routines that build on the low-level motor
 * helpers defined in rover_interpreter.ino.
 *
 * All .ino files in the same sketch folder are compiled together, so
 * everything defined in rover_interpreter.ino (pins, constants, encoder
 * state, setMotor, stopMotors, readGroundSensor, flipped, etc.) is
 * directly accessible here without any includes.
 *
 * Currently implemented:
 *   handleStraight — encoder-differential heading hold (SYS_STRAIGHT = 6)
 */

// ── Straight drive tuning ─────────────────────────────────────────────────────
// P-controller gain for straight-line correction.
// Increase if rover still drifts, decrease if it oscillates side to side.
#define STRAIGHT_KP             4.0f
#define STRAIGHT_MAX_CORRECTION 60    // max PWM adjustment per side (0–255)

// ── handleStraight ────────────────────────────────────────────────────────────
// Drives forward (DIR_UP) or backward (DIR_DOWN) for distanceCm while a
// P-controller continuously compares left and right encoder counts and trims
// each side's PWM to keep them equal, compensating for motor imbalance and
// mechanical drift.  No IMU needed — pure encoder feedback.
//
// Sends "DONE:<left_cm>,<right_cm>,<left_ratio>,<right_ratio>" on completion,
// or "CLIFF:<left_cm>,<right_cm>" if the ground sensor loses the floor.
// Same response format as handleMotor so Pi code needs no changes.
void handleStraight(int direction, float distanceCm, float speed) {
    if (distanceCm <= 0.0f) return;  // must have a target distance

    int basePwm = constrain((int)(speed * 255.0f), 0, 255);

    int effectiveDir = direction;
    if (flipped) {
        if      (effectiveDir == DIR_UP)   effectiveDir = DIR_DOWN;
        else if (effectiveDir == DIR_DOWN) effectiveDir = DIR_UP;
    }
    bool forward = (effectiveDir == DIR_UP);

    noInterrupts();
    long startLeft  = encoderCountLeft;
    long startRight = encoderCountRight;
    interrupts();

    long targetCounts = (long)(distanceCm / DIST_PER_REV_CM * COUNTS_PER_REV);
    unsigned long deadline = millis() + MOTOR_TIMEOUT_MS;
    bool cliffDetected = false;

    // Kick motors off at base speed before entering control loop
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, basePwm, forward);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, basePwm, forward);

    while (millis() < deadline) {
        noInterrupts();
        long rawL = encoderCountLeft  - startLeft;
        long rawR = encoderCountRight - startRight;
        interrupts();

        long absL = abs(rawL);
        long absR = abs(rawR);

        if (absL >= targetCounts || absR >= targetCounts) break;

        if (readGroundSensor() > CLIFF_THRESHOLD_CM) {
            cliffDetected = true;
            break;
        }

        // P-controller: positive error means left is ahead → slow left, speed right
        long diff       = absL - absR;
        int  correction = (int)(STRAIGHT_KP * (float)diff);
        correction = constrain(correction, -STRAIGHT_MAX_CORRECTION, STRAIGHT_MAX_CORRECTION);

        int leftPwm  = constrain(basePwm - correction, 0, 255);
        int rightPwm = constrain(basePwm + correction, 0, 255);

        setMotor(PIN_ENA, PIN_IN1, PIN_IN2, leftPwm,  forward);
        setMotor(PIN_ENB, PIN_IN3, PIN_IN4, rightPwm, forward);

        delay(10);  // 100 Hz control loop
    }

    stopMotors();

    noInterrupts();
    long actualLeft  = abs(encoderCountLeft  - startLeft);
    long actualRight = abs(encoderCountRight - startRight);
    interrupts();

    float leftCm  = (float)actualLeft  / COUNTS_PER_REV * DIST_PER_REV_CM;
    float rightCm = (float)actualRight / COUNTS_PER_REV * DIST_PER_REV_CM;

    if (cliffDetected) {
        Serial.print("CLIFF:");
        Serial.print(leftCm);
        Serial.print(",");
        Serial.println(rightCm);
        return;
    }

    float leftRatio  = (targetCounts > 0) ? (float)actualLeft  / targetCounts : 1.0f;
    float rightRatio = (targetCounts > 0) ? (float)actualRight / targetCounts : 1.0f;
    Serial.print("DONE:");
    Serial.print(leftCm);        Serial.print(",");
    Serial.print(rightCm);       Serial.print(",");
    Serial.print(leftRatio,  4); Serial.print(",");
    Serial.println(rightRatio, 4);
}

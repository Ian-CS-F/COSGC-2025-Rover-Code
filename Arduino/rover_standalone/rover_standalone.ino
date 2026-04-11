#include <Arduino.h>
#include <Servo.h>
// Wire.h included by imu.ino, math.h included by heightmap.ino

/*
 * rover_standalone.ino
 * ====================
 * Autonomous rover operation WITHOUT a Raspberry Pi.
 * Upload rover_standalone/ when the Pi is unavailable.
 *
 * Companion files (compiled together automatically):
 *   imu.ino        — ICM-20948 gyro heading integration
 *   heightmap.ino  — ultrasonic tilt-sweep heightmap and obstacle/cliff detection
 *
 * Behaviour loop:
 *   1. Sweep tilt servo → build ultrasonic heightmap
 *   2. Obstacle detected in heightmap → tank-turn AVOID_TURN_DEG, retry (max 6 times)
 *   3. Cliff detected in heightmap → halt permanently
 *   4. Drive SEGMENT_CM with IMU gyro heading hold (encoder fallback if IMU absent)
 *   5. Repeat
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   Left motor    : L298N A  ENA=4, IN1=6, IN2=7
 *   Right motor   : L298N B  ENB=5, IN3=8, IN4=9
 *   Tilt servo 1  : pin 12
 *   Tilt servo 2  : pin 13  (mirror-mounted — commanded 180−angle)
 *   Ultrasonic    : TRIG=A0, ECHO=A1  (on tilt servo bracket)
 *   Left encoder  : A=pin2 (INT0), B=pin10
 *   Right encoder : A=pin3 (INT1), B=pin11
 *   ICM-20948 IMU : SDA=pin20, SCL=pin21, VCC=3.3V, AD0=3.3V (addr 0x69)
 *                   Use a level shifter if your breakout has no onboard one.
 *
 * ── Tuning ───────────────────────────────────────────────────────────────────
 * SEGMENT_CM            distance driven per check cycle
 * DRIVE_SPEED           motor speed 0.0–1.0
 * START_DELAY_MS        countdown before moving (seconds to place rover)
 * AVOID_TURN_DEG        degrees to tank-turn when blocked
 * MAX_AVOID_ATTEMPTS    give up and halt after this many turns in a row
 * IMU_KP                heading correction gain (IMU) — raise if drifts
 * ENCODER_KP            heading correction gain (encoder fallback)
 * MAX_CORRECTION        max PWM trim per side (0–255)
 * MS_PER_DEGREE         ms per degree of tank turn — calibrate on your rover
 *
 * Heightmap tuning is in heightmap.ino.
 * IMU register details are in imu.ino.
 */

// ── Drive configuration ───────────────────────────────────────────────────────
#define SEGMENT_CM           30.0f
#define DRIVE_SPEED           0.65f  // >0.6 needed for reliable motor operation
#define START_DELAY_MS        3000

// ── Obstacle avoidance ────────────────────────────────────────────────────────
// OBSTACLE_THRESHOLD_CM is also used by heightmap.ino (compiled together)
#define OBSTACLE_THRESHOLD_CM 60.0f   // stop and avoid if obstacle closer than this
#define AVOID_TURN_DEG        30.0f
#define AVOID_TURN_PWM        170     // >0.6 of 255 — matches minimum reliable speed
#define MS_PER_DEGREE         12      // ms per degree of tank turn — calibrate!

// ── Cliff detection ───────────────────────────────────────────────────────────
#define CLIFF_TILT_DEG        140     // servo angle for downward cliff check
#define CLIFF_TILT_SETTLE_MS  300     // ms to wait for servo to reach angle
#define CLIFF_THRESHOLD_CM    40.0f   // ultrasonic: floor missing if reading > this

// ── Heading hold ──────────────────────────────────────────────────────────────
#define IMU_KP               8.0f    // gain when IMU available
#define ENCODER_KP           4.0f    // gain when falling back to encoders
#define MAX_CORRECTION       60      // max PWM trim per side

// ── Pin definitions ───────────────────────────────────────────────────────────
#define PIN_ENA  4
#define PIN_IN1  6
#define PIN_IN2  7
#define PIN_ENB  5
#define PIN_IN3  8
#define PIN_IN4  9

#define PIN_TILT  12
#define PIN_TILT2 13

#define PIN_ULTRA_TRIG A0
#define PIN_ULTRA_ECHO A1

#define ENCODER_A_LEFT  2
#define ENCODER_B_LEFT  10
#define ENCODER_A_RIGHT 3
#define ENCODER_B_RIGHT 11

// ── Constants ─────────────────────────────────────────────────────────────────
#define COUNTS_PER_REV   64
#define DIST_PER_REV_CM  35.2f
#define MOTOR_TIMEOUT_MS 30000UL

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long encoderCountLeft  = 0;
volatile long encoderCountRight = 0;

void encoderISR_Left() {
    if (digitalRead(ENCODER_B_LEFT))  encoderCountLeft++;
    else                               encoderCountLeft--;
}

void encoderISR_Right() {
    if (digitalRead(ENCODER_B_RIGHT)) encoderCountRight--;
    else                               encoderCountRight++;
}

// ── Servos ────────────────────────────────────────────────────────────────────
Servo tiltServo;
Servo tiltServo2;

void setTilt(int angle) {
    angle = constrain(angle, 0, 180);
    tiltServo.write(angle);
    tiltServo2.write(180 - angle);
}

// ── Motor helpers ─────────────────────────────────────────────────────────────
void setMotor(int en, int in1, int in2, int pwm, bool forward) {
    digitalWrite(in1, forward ? HIGH : LOW);
    digitalWrite(in2, forward ? LOW  : HIGH);
    analogWrite(en, pwm);
}

void stopMotors() {
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
}

// ── Ultrasonic ────────────────────────────────────────────────────────────────
float readUltrasonic() {
    digitalWrite(PIN_ULTRA_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRA_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRA_TRIG, LOW);
    long us = pulseIn(PIN_ULTRA_ECHO, HIGH, 30000UL);
    return (us == 0) ? 999.0f : us * 0.01715f;
}

// ── Tank turns ────────────────────────────────────────────────────────────────
void turnRight(float degrees) {
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, AVOID_TURN_PWM, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, AVOID_TURN_PWM, false);
    delay((int)(degrees * MS_PER_DEGREE));
    stopMotors();
}

void turnLeft(float degrees) {
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, AVOID_TURN_PWM, false);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, AVOID_TURN_PWM, true);
    delay((int)(degrees * MS_PER_DEGREE));
    stopMotors();
}

// ── Straight drive with heading hold ──────────────────────────────────────────
// Uses IMU gyro if available, encoder differential as fallback.
// imuAvailable is set in setup() after imuBegin().
static bool imuAvailable   = false;
static bool lidarAvailable = false;

void driveStraight(float distanceCm, float speed) {
    int  basePwm      = constrain((int)(speed * 255.0f), 0, 255);
    long targetCounts = (long)(distanceCm / DIST_PER_REV_CM * COUNTS_PER_REV);

    noInterrupts();
    long startLeft  = encoderCountLeft;
    long startRight = encoderCountRight;
    interrupts();

    imuResetHeading();
    unsigned long deadline = millis() + MOTOR_TIMEOUT_MS;

    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, basePwm, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, basePwm, true);

    while (millis() < deadline) {
        noInterrupts();
        long absL = abs(encoderCountLeft  - startLeft);
        long absR = abs(encoderCountRight - startRight);
        interrupts();

        if (absL >= targetCounts || absR >= targetCounts) break;

        imuUpdate();

        int correction;
        if (imuAvailable) {
            // IMU: positive heading = drifted right → slow right, speed left
            correction = constrain((int)(IMU_KP * imuHeadingDeg()),
                                   -MAX_CORRECTION, MAX_CORRECTION);
        } else {
            // Encoder fallback: positive diff = left ahead → need to slow left.
            // Negate so that positive diff → negative correction → left slows.
            long diff  = absL - absR;
            correction = constrain((int)(-ENCODER_KP * diff),
                                   -MAX_CORRECTION, MAX_CORRECTION);
        }

        // Positive correction → left motor faster, right slower (turns right).
        // IMU:     +heading = turned right → correction positive → speeds left back ✓
        // Encoder: left ahead → correction negative → slows left ✓
        setMotor(PIN_ENA, PIN_IN1, PIN_IN2,
                 constrain(basePwm + correction, 0, 255), true);
        setMotor(PIN_ENB, PIN_IN3, PIN_IN4,
                 constrain(basePwm - correction, 0, 255), true);

        delay(10);
    }

    stopMotors();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);

    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    stopMotors();

    pinMode(PIN_ULTRA_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_ECHO, INPUT);

    pinMode(ENCODER_A_LEFT,  INPUT_PULLUP);
    pinMode(ENCODER_B_LEFT,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT),  encoderISR_Left,  RISING);

    pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
    pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), encoderISR_Right, RISING);

    tiltServo.attach(PIN_TILT);
    tiltServo2.attach(PIN_TILT2);
    setTilt(90);

    imuAvailable = imuBegin();
    if (!imuAvailable) {
        Serial.println("IMU not found — using encoder heading hold as fallback.");
    }

    lidarAvailable = lidarBegin();
    if (!lidarAvailable) {
        Serial.println("LIDAR not found — using heightmap for obstacle detection.");
    }

    Serial.print("STANDALONE: starting in ");
    Serial.print(START_DELAY_MS / 1000);
    Serial.println(" seconds...");
    delay(START_DELAY_MS);
    Serial.println("STANDALONE: running.");
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    // ── 1. Heightmap sweep (populates data used by chooseDirection) ───────────
    hmScan();
    hmPrint();

    // ── 2. Zone-based navigation decision ─────────────────────────────────────
    int dir = chooseDirection();

    if (dir == NAV_BACK) {
        Serial.println("STANDALONE: all zones blocked — reversing 180°.");
        turnRight(180.0f);
        delay(200);
        return;  // rescan immediately after turning
    }

    if (dir == NAV_LEFT) {
        Serial.print("STANDALONE: turning left ");
        Serial.print(AVOID_TURN_DEG); Serial.println("°");
        turnLeft(AVOID_TURN_DEG);
    } else if (dir == NAV_RIGHT) {
        Serial.print("STANDALONE: turning right ");
        Serial.print(AVOID_TURN_DEG); Serial.println("°");
        turnRight(AVOID_TURN_DEG);
    }
    // NAV_CENTER: no turn needed

    // ── 3. Cliff check before driving ─────────────────────────────────────────
    // Tilt down, read ultrasonic, tilt back to level
    setTilt(CLIFF_TILT_DEG);
    delay(CLIFF_TILT_SETTLE_MS);
    float groundDist = readUltrasonic();
    setTilt(90);
    Serial.print("CLIFF: "); Serial.print(groundDist); Serial.println(" cm");

    if (groundDist > CLIFF_THRESHOLD_CM) {
        Serial.println("STANDALONE: cliff detected — halted.");
        while (true) {}
    }

    // ── 4. Drive one segment with heading hold ────────────────────────────────
    Serial.print("STANDALONE: driving "); Serial.print(SEGMENT_CM); Serial.println(" cm");
    driveStraight(SEGMENT_CM, DRIVE_SPEED);
    delay(100);
}

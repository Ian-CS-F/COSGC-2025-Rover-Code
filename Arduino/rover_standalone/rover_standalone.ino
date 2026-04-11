#include <Arduino.h>
#include <Servo.h>

/*
 * rover_standalone.ino
 * ====================
 * Autonomous rover operation WITHOUT a Raspberry Pi.
 * Upload this sketch when the Pi is unavailable.
 *
 * Single HC-SR04 ultrasonic mounted on the tilt servo bracket.
 * The servo aims it forward for obstacle detection, then down for
 * cliff detection, before each drive segment.
 *
 * Behaviour loop:
 *   1. Tilt forward → read ultrasonic
 *      - Obstacle within OBSTACLE_THRESHOLD_CM → turn AVOID_TURN_DEG and retry
 *   2. Tilt down → read ultrasonic
 *      - Ground too far (> CLIFF_THRESHOLD_CM) → stop permanently
 *   3. Drive SEGMENT_CM straight with encoder heading hold
 *   4. Repeat
 *
 * ── Tuning ───────────────────────────────────────────────────────────────────
 * SEGMENT_CM            distance driven per check cycle (cm)
 * DRIVE_SPEED           motor speed 0.0–1.0
 * START_DELAY_MS        countdown before moving (ms) — time to place rover
 * TILT_FORWARD_DEG      servo angle for forward-facing scan (obstacle)
 * TILT_CLIFF_DEG        servo angle for downward-facing scan (cliff)
 *                         increase if ultrasonic doesn't see the floor
 * TILT_SETTLE_MS        wait after moving servo before reading (ms)
 * OBSTACLE_THRESHOLD_CM stop and turn if obstacle closer than this (cm)
 * CLIFF_THRESHOLD_CM    stop if floor further than this (cm)
 * AVOID_TURN_DEG        degrees to tank-turn when obstacle detected
 * STRAIGHT_KP           P-gain for heading hold; raise if drifts, lower if oscillates
 *
 * Hardware: Arduino Mega 2560
 *   Left motor   : L298N A  (ENA=4, IN1=6, IN2=7)
 *   Right motor  : L298N B  (ENB=5, IN3=8, IN4=9)
 *   Tilt servo 1 : pin 12
 *   Tilt servo 2 : pin 13  (mirror-mounted — always commanded 180-angle)
 *   Ultrasonic   : TRIG=A0, ECHO=A1  (mounted on servo bracket)
 *   Left encoder : A=pin2 (INT0), B=pin10
 *   Right encoder: A=pin3 (INT1), B=pin11
 */

// ── Course / behaviour configuration ─────────────────────────────────────────
#define SEGMENT_CM              30.0f   // drive this far between sensor checks
#define DRIVE_SPEED              0.55f
#define START_DELAY_MS           3000

// ── Servo / sensor angles ─────────────────────────────────────────────────────
#define TILT_FORWARD_DEG         90     // level — looks straight ahead
#define TILT_CLIFF_DEG          140     // angled down — looks at ground ahead
#define TILT_SETTLE_MS          300     // ms to wait after moving servo

// ── Detection thresholds ──────────────────────────────────────────────────────
#define OBSTACLE_THRESHOLD_CM    50.0f  // obstacle if reading < this
#define CLIFF_THRESHOLD_CM       40.0f  // cliff if reading > this

// ── Obstacle avoidance ────────────────────────────────────────────────────────
#define AVOID_TURN_DEG           30.0f  // degrees to turn when blocked
#define AVOID_TURN_PWM           150    // motor PWM during turn
#define MS_PER_DEGREE            12     // ms per degree of tank turn — calibrate!
#define MAX_AVOID_ATTEMPTS        6     // give up after this many turns in a row

// ── Heading hold tuning ───────────────────────────────────────────────────────
#define STRAIGHT_KP              4.0f
#define STRAIGHT_MAX_CORRECTION  60

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
    tiltServo2.write(180 - angle);  // mirror-mounted
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

// Read ultrasonic at a given tilt angle (moves servo, waits, reads, leaves servo there)
float readAt(int tiltDeg) {
    setTilt(tiltDeg);
    delay(TILT_SETTLE_MS);
    return readUltrasonic();
}

// ── Tank turns ────────────────────────────────────────────────────────────────
void turnRight(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, AVOID_TURN_PWM, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, AVOID_TURN_PWM, false);
    delay(ms);
    stopMotors();
}

void turnLeft(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, AVOID_TURN_PWM, false);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, AVOID_TURN_PWM, true);
    delay(ms);
    stopMotors();
}

// ── Straight drive with encoder heading hold ──────────────────────────────────
// Returns true = target reached, false = timed out
bool driveStraight(float distanceCm, float speed) {
    int  basePwm      = constrain((int)(speed * 255.0f), 0, 255);
    long targetCounts = (long)(distanceCm / DIST_PER_REV_CM * COUNTS_PER_REV);

    noInterrupts();
    long startLeft  = encoderCountLeft;
    long startRight = encoderCountRight;
    interrupts();

    unsigned long deadline = millis() + MOTOR_TIMEOUT_MS;

    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, basePwm, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, basePwm, true);

    while (millis() < deadline) {
        noInterrupts();
        long absL = abs(encoderCountLeft  - startLeft);
        long absR = abs(encoderCountRight - startRight);
        interrupts();

        if (absL >= targetCounts || absR >= targetCounts) break;

        long diff       = absL - absR;
        int  correction = constrain((int)(STRAIGHT_KP * diff),
                                    -STRAIGHT_MAX_CORRECTION,
                                     STRAIGHT_MAX_CORRECTION);

        setMotor(PIN_ENA, PIN_IN1, PIN_IN2,
                 constrain(basePwm - correction, 0, 255), true);
        setMotor(PIN_ENB, PIN_IN3, PIN_IN4,
                 constrain(basePwm + correction, 0, 255), true);

        delay(10);
    }

    stopMotors();
    return true;
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
    setTilt(TILT_FORWARD_DEG);

    Serial.print("STANDALONE: starting in ");
    Serial.print(START_DELAY_MS / 1000);
    Serial.println(" seconds...");
    delay(START_DELAY_MS);
    Serial.println("STANDALONE: running.");
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    // ── 1. Obstacle check (servo forward) ─────────────────────────────────────
    int avoidAttempts = 0;
    while (avoidAttempts < MAX_AVOID_ATTEMPTS) {
        float forwardDist = readAt(TILT_FORWARD_DEG);
        Serial.print("OBSTACLE CHECK: ");
        Serial.print(forwardDist);
        Serial.println(" cm");

        if (forwardDist >= OBSTACLE_THRESHOLD_CM) break;  // clear

        // Blocked — turn right and try again
        Serial.print("OBSTACLE: turning right ");
        Serial.print(AVOID_TURN_DEG);
        Serial.println(" deg");
        turnRight(AVOID_TURN_DEG);
        delay(200);
        avoidAttempts++;
    }

    if (avoidAttempts >= MAX_AVOID_ATTEMPTS) {
        Serial.println("STANDALONE: stuck — cannot find clear path. Stopped.");
        while (true) {}  // halt
    }

    // ── 2. Cliff check (servo down) ───────────────────────────────────────────
    float groundDist = readAt(TILT_CLIFF_DEG);
    Serial.print("CLIFF CHECK: ");
    Serial.print(groundDist);
    Serial.println(" cm");

    if (groundDist > CLIFF_THRESHOLD_CM) {
        Serial.println("STANDALONE: cliff detected — stopped.");
        setTilt(TILT_FORWARD_DEG);
        while (true) {}  // halt
    }

    // ── 3. Drive one segment ──────────────────────────────────────────────────
    setTilt(TILT_FORWARD_DEG);  // return to forward while driving
    Serial.print("STANDALONE: driving ");
    Serial.print(SEGMENT_CM);
    Serial.println(" cm");
    driveStraight(SEGMENT_CM, DRIVE_SPEED);
    delay(100);  // brief pause before next check
}

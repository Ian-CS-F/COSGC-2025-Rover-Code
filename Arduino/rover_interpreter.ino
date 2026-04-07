#include <Arduino.h>
#include <Servo.h>

/*
 * Communication Schema  (Pi → Arduino)
 * (System, Direction, Amount, Speed)
 *
 * System:    Servo (0), Motor (1), Sweep (2), Query (3), Flip (4)
 * Direction: left (0), right (1), up (2), down (3)
 *            — ignored for Sweep and Query
 * Amount:    Servo: target angle as 0.0–1.0 (→ 0–180°)
 *            Motor: target distance in cm  (0 = continuous, no distance control)
 *            Sweep: total angular range in degrees (e.g. 90.0)
 * Speed:     Motor: PWM duty cycle 0.0–1.0 (→ 0–255)
 *            Sweep: time between steps in ms (e.g. 200.0)
 *
 * Motor distance protocol:
 *   Pi sends      (1, <dir>, <distance_cm>, <speed>)
 *   Arduino drives until encoder reaches target (or timeout), then stops
 *   Arduino sends "DONE:<left_cm>,<right_cm>,<left_ratio>,<right_ratio>"
 *     left_cm / right_cm     — encoder-measured distance per side
 *     left_ratio/right_ratio — actual_counts / target_counts per side
 *                   > 1.0 : wheels spun more than expected (slipping)
 *                   < 1.0 : fewer counts than expected (stalled / very slow)
 *   (If distance_cm == 0, motors run continuously; no DONE is sent)
 *
 * Sweep protocol:
 *   Pi sends      (2, 0, <range_deg>, <step_ms>)
 *   At each horizontal position the tilt servo sweeps up then back down
 *   through TILT_ANGLES[], pausing at every angle for the Pi to read LIDAR
 *   Arduino sends "AT:<h_angle>,<tilt_angle>"  at each tilt position
 *   Pi sends      "NEXT"                        when ready for the next position
 *   Arduino sends "SWEEP_DONE"                  when back at centre (angle 0)
 *
 * Query protocol:
 *   Pi sends      (3, 0, 0, 0)
 *   Arduino sends "DIST:<left_cm>,<right_cm>"  — distance per side since last query, then resets counters
 *
 * Cliff detection (Arduino → Pi, unsolicited):
 *   Arduino sends "CLIFF:<left_cm>,<right_cm>"  when the downward ultrasonic loses the
 *   ground mid-drive.  Motors are already stopped when this is sent.
 *   left_cm/right_cm are the partial encoder distances covered before the stop.
 *   No DONE is sent for that segment.
 *
 * Flip protocol:
 *   Pi sends      (4, 0, 1.0, 0)  — enter flipped mode
 *   Pi sends      (4, 0, 0.0, 0)  — return to normal mode
 *   When flipped: motor directions invert, ground ultrasonic switches to upward sensor
 *
 * Handshake:
 *   Arduino sends  "READY"  every second until Pi replies "ACK"
 *
 * Hardware: Arduino Mega 2560
 */

// ── Pin definitions ───────────────────────────────────────────────────────────
// Left motor (L298N channel A)
#define PIN_ENA  4   // PWM — Timer0
#define PIN_IN1  6
#define PIN_IN2  7

// Right motor (L298N channel B)
#define PIN_ENB  5   // PWM — Timer3
#define PIN_IN3  8
#define PIN_IN4  9

#define PIN_TILT 11  // Camera tilt servo

// Downward-facing ultrasonic (HC-SR04) — cliff detection (normal mode)
#define PIN_ULTRA_DOWN_TRIG A0
#define PIN_ULTRA_DOWN_ECHO A1

// Upward-facing ultrasonic (HC-SR04) — cliff detection when inverted
#define PIN_ULTRA_UP_TRIG A2
#define PIN_ULTRA_UP_ECHO A3

// Left encoder — hardware interrupt INT0
#define ENCODER_A_LEFT  2   // INT0
#define ENCODER_B_LEFT  10  // direction read (any digital pin)

// Right encoder — hardware interrupt INT1
#define ENCODER_A_RIGHT 3   // INT1
#define ENCODER_B_RIGHT 12  // direction read (any digital pin)

// ── Constants ─────────────────────────────────────────────────────────────────
#define BAUD_RATE 9600

#define MIDDLE_ANGLE 90

#define SYS_SERVO 0
#define SYS_MOTOR 1
#define SYS_SWEEP 2
#define SYS_QUERY 3
#define SYS_FLIP  4

#define COUNTS_PER_REV   64     // 64 P/R, 1x decoding (rising edge on A only)
#define DIST_PER_REV_CM  10.0f  // cm per full encoder revolution — calibrate!

#define DIR_LEFT  0
#define DIR_RIGHT 1
#define DIR_UP    2
#define DIR_DOWN  3

#define SWEEP_STEP_DEG   5
#define SWEEP_TURN_PWM   150
#define MS_PER_DEGREE    12    // ms per degree — calibrate!

#define MOTOR_TIMEOUT_MS 30000UL

#define CLIFF_THRESHOLD_CM    40.0f
#define CLIFF_ALERT_INTERVAL  1000UL

// Tilt angles visited at each horizontal position (up then back down)
const int TILT_ANGLES[] = {60, 75, 90, 105, 120};
const int TILT_COUNT    = sizeof(TILT_ANGLES) / sizeof(TILT_ANGLES[0]);

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long encoderCountLeft  = 0;
volatile long encoderCountRight = 0;

void encoderISR_Left() {
    if (digitalRead(ENCODER_B_LEFT))  encoderCountLeft++;
    else                               encoderCountLeft--;
}

void encoderISR_Right() {
    if (digitalRead(ENCODER_B_RIGHT)) encoderCountRight++;
    else                               encoderCountRight--;
}

// ── Servo ─────────────────────────────────────────────────────────────────────
Servo tiltServo;

// ── Packet parsing ────────────────────────────────────────────────────────────
struct Command {
    int   system;
    int   direction;
    float amount;
    float speed;
};

bool parsePacket(const String& raw, Command& cmd) {
    int start = raw.indexOf('(');
    int end   = raw.indexOf(')');
    if (start == -1 || end == -1 || end <= start) return false;

    String body = raw.substring(start + 1, end);
    int i0 = body.indexOf(',');
    int i1 = body.indexOf(',', i0 + 1);
    int i2 = body.indexOf(',', i1 + 1);
    if (i0 == -1 || i1 == -1 || i2 == -1) return false;

    cmd.system    = body.substring(0,      i0).toInt();
    cmd.direction = body.substring(i0 + 1, i1).toInt();
    cmd.amount    = body.substring(i1 + 1, i2).toFloat();
    cmd.speed     = body.substring(i2 + 1    ).toFloat();
    return true;
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

// ── Flip state ────────────────────────────────────────────────────────────────
bool flipped = false;

// ── Ultrasonic ────────────────────────────────────────────────────────────────
float readUltrasonicDown() {
    digitalWrite(PIN_ULTRA_DOWN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRA_DOWN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRA_DOWN_TRIG, LOW);
    long us = pulseIn(PIN_ULTRA_DOWN_ECHO, HIGH, 30000UL);
    return (us == 0) ? 999.0f : us * 0.01715f;
}

float readUltrasonicUp() {
    digitalWrite(PIN_ULTRA_UP_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRA_UP_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRA_UP_TRIG, LOW);
    long us = pulseIn(PIN_ULTRA_UP_ECHO, HIGH, 30000UL);
    return (us == 0) ? 999.0f : us * 0.01715f;
}

float readGroundSensor() {
    return flipped ? readUltrasonicUp() : readUltrasonicDown();
}

// ── Turn helpers ──────────────────────────────────────────────────────────────
void turnLeft(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, SWEEP_TURN_PWM, false);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, SWEEP_TURN_PWM, true);
    delay(ms);
    stopMotors();
}

void turnRight(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, SWEEP_TURN_PWM, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, SWEEP_TURN_PWM, false);
    delay(ms);
    stopMotors();
}

// ── Sweep ─────────────────────────────────────────────────────────────────────
void tiltStep(float hAngle, int tiltAngle, int stepMs) {
    tiltServo.write(tiltAngle);
    Serial.print("AT:");
    Serial.print(hAngle);
    Serial.print(",");
    Serial.println(tiltAngle - MIDDLE_ANGLE);

    unsigned long deadline = millis() + stepMs;
    while (millis() < deadline) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line == "NEXT") return;
        }
    }
}

void sweep(float range, int stepMs) {
    float halfRange = range / 2.0f;
    int   numSteps  = (int)(range / SWEEP_STEP_DEG);

    turnLeft(halfRange);

    for (int i = 0; i <= numSteps; i++) {
        float hAngle = -halfRange + i * SWEEP_STEP_DEG;

        for (int t = 0; t < TILT_COUNT; t++)
            tiltStep(hAngle, TILT_ANGLES[t], stepMs);

        for (int t = TILT_COUNT - 2; t >= 0; t--)
            tiltStep(hAngle, TILT_ANGLES[t], stepMs);

        if (i < numSteps)
            turnRight(SWEEP_STEP_DEG);
    }

    tiltServo.write(90);
    turnLeft(halfRange);
    Serial.println("SWEEP_DONE");
}

// ── Handlers ──────────────────────────────────────────────────────────────────
void handleServo(int direction, float amount, float /*speed*/) {
    int angle = constrain((int)(amount * 180.0f), 0, 180);
    switch (direction) {
        case DIR_UP:
        case DIR_DOWN:
            tiltServo.write(angle);
            break;
    }
}

void handleMotor(int direction, float distanceCm, float speed) {
    int pwm = constrain((int)(speed * 255.0f), 0, 255);

    int effectiveDir = direction;
    if (flipped) {
        if      (effectiveDir == DIR_UP)    effectiveDir = DIR_DOWN;
        else if (effectiveDir == DIR_DOWN)  effectiveDir = DIR_UP;
        else if (effectiveDir == DIR_LEFT)  effectiveDir = DIR_RIGHT;
        else if (effectiveDir == DIR_RIGHT) effectiveDir = DIR_LEFT;
    }

    switch (effectiveDir) {
        case DIR_UP:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, true);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, true);
            break;
        case DIR_DOWN:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, false);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, false);
            break;
        case DIR_LEFT:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, false);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, true);
            break;
        case DIR_RIGHT:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, true);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, false);
            break;
        default:
            stopMotors();
            return;
    }

    if (distanceCm <= 0.0f) return;  // continuous mode

    noInterrupts();
    long startLeft  = encoderCountLeft;
    long startRight = encoderCountRight;
    interrupts();

    long targetCounts = (long)(distanceCm / DIST_PER_REV_CM * COUNTS_PER_REV);
    unsigned long deadline = millis() + MOTOR_TIMEOUT_MS;
    bool cliffDetected = false;

    while (millis() < deadline) {
        noInterrupts();
        long tL = abs(encoderCountLeft  - startLeft);
        long tR = abs(encoderCountRight - startRight);
        interrupts();

        if (tL >= targetCounts || tR >= targetCounts) break;

        if (readGroundSensor() > CLIFF_THRESHOLD_CM) {
            cliffDetected = true;
            break;
        }
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
    Serial.print(leftCm);   Serial.print(",");
    Serial.print(rightCm);  Serial.print(",");
    Serial.print(leftRatio,  4); Serial.print(",");
    Serial.println(rightRatio, 4);
}

void handleSweep(float amount, float speed) {
    sweep(amount, (int)speed);
}

void handleQuery() {
    noInterrupts();
    long countLeft  = encoderCountLeft;
    long countRight = encoderCountRight;
    encoderCountLeft  = 0;
    encoderCountRight = 0;
    interrupts();

    float leftCm  = (float)countLeft  / COUNTS_PER_REV * DIST_PER_REV_CM;
    float rightCm = (float)countRight / COUNTS_PER_REV * DIST_PER_REV_CM;
    Serial.print("DIST:");
    Serial.print(leftCm);  Serial.print(",");
    Serial.println(rightCm);
}

void handleFlip(float amount) {
    flipped = (amount >= 0.5f);
}

void dispatch(const Command& cmd) {
    switch (cmd.system) {
        case SYS_SERVO: handleServo(cmd.direction, cmd.amount, cmd.speed); break;
        case SYS_MOTOR: handleMotor(cmd.direction, cmd.amount, cmd.speed); break;
        case SYS_SWEEP: handleSweep(cmd.amount, cmd.speed);                break;
        case SYS_QUERY: handleQuery();                                      break;
        case SYS_FLIP:  handleFlip(cmd.amount);                            break;
    }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
unsigned long lastCliffAlert = 0;

void setup() {
    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    stopMotors();

    pinMode(PIN_ULTRA_DOWN_TRIG, OUTPUT); pinMode(PIN_ULTRA_DOWN_ECHO, INPUT);
    pinMode(PIN_ULTRA_UP_TRIG,   OUTPUT); pinMode(PIN_ULTRA_UP_ECHO,   INPUT);

    // Both encoders use hardware interrupts on the Mega (INT0 and INT1)
    pinMode(ENCODER_A_LEFT,  INPUT_PULLUP);
    pinMode(ENCODER_B_LEFT,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT),  encoderISR_Left,  RISING);

    pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
    pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), encoderISR_Right, RISING);

    tiltServo.attach(PIN_TILT);
    tiltServo.write(90);

    Serial.begin(BAUD_RATE);
    unsigned long lastSent = 0;
    while (true) {
        if (millis() - lastSent >= 1000) {
            Serial.println("READY");
            lastSent = millis();
        }
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line == "ACK") break;
        }
    }
}

void loop() {
    if (millis() - lastCliffAlert >= CLIFF_ALERT_INTERVAL) {
        float groundDist = readGroundSensor();
        if (groundDist > CLIFF_THRESHOLD_CM) {
            stopMotors();
            Serial.print("CLIFF:");
            Serial.println(groundDist);
            lastCliffAlert = millis();
        }
    }

    if (Serial.available()) {
        String raw = Serial.readStringUntil('\n');
        Command cmd;
        if (parsePacket(raw, cmd)) {
            dispatch(cmd);
        }
    }
}

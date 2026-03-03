#include <Arduino.h>
#include <Servo.h>

/*
 * Communication Schema  (Pi → Arduino)
 * (System, Direction, Amount, Speed)
 *
 * System:    Servo (0), Motor (1), Sweep (2), Query (3)
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
 *   Arduino sends "DONE:<actual_cm>,<error_ratio>"
 *     actual_cm   — encoder-measured distance covered
 *     error_ratio — actual_counts / target_counts
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
 *   Arduino sends "DIST:<cm>"  — distance since last query, then resets counter
 *
 * Cliff detection (Arduino → Pi, unsolicited):
 *   Arduino sends "CLIFF:<dist_cm>"  when the downward ultrasonic loses the
 *   ground mid-drive.  Motors are already stopped when this is sent.
 *   dist_cm is the partial encoder distance covered before the stop.
 *   No DONE is sent for that segment.
 *
 * Handshake:
 *   Arduino sends  "READY"  every second until Pi replies "ACK"
 */

// ── Pin definitions ───────────────────────────────────────────────────────────
#define PIN_ENA  5   // PWM  — left motor
#define PIN_IN1  6
#define PIN_IN2  7

#define PIN_ENB  10  // PWM  — right motor
#define PIN_IN3  8
#define PIN_IN4  9

#define PIN_TILT 11  // Camera tilt servo

// Downward-facing ultrasonic (HC-SR04) — cliff / drop-off detection
#define PIN_ULTRA_DOWN_TRIG A0
#define PIN_ULTRA_DOWN_ECHO A1

// Encoder (SparkFun 64 P/R quadrature) — hardware interrupt pins
#define ENCODER_A 2  // interrupt pin — channel A
#define ENCODER_B 3  // direction pin — channel B

// ── Constants ─────────────────────────────────────────────────────────────────
#define BAUD_RATE 9600

#define MIDDLE_ANGLE 90 //angle of the servo for straight forward

#define SYS_SERVO 0
#define SYS_MOTOR 1
#define SYS_SWEEP 2
#define SYS_QUERY 3

// Encoder odometry — calibrate DIST_PER_REV_CM for your track
#define COUNTS_PER_REV   64     // 64 P/R, 1x decoding (rising edge on A)
#define DIST_PER_REV_CM  10.0f  // cm per full encoder revolution — measure and update!

#define DIR_LEFT  0
#define DIR_RIGHT 1
#define DIR_UP    2
#define DIR_DOWN  3

// Sweep rotation
#define SWEEP_STEP_DEG   5     // horizontal angular resolution per step (degrees)
#define SWEEP_TURN_PWM   150   // motor PWM used during sweep rotation
#define MS_PER_DEGREE    12    // ms of motor runtime per degree — calibrate!

// Distance-controlled driving
#define MOTOR_TIMEOUT_MS 30000UL  // 30 s hard timeout per drive segment

// Cliff detection — downward ultrasonic at 45°
// On flat ground the sensor reads ~sensor_height / sin(45°).
// If it suddenly reads above CLIFF_THRESHOLD_CM, the ground has dropped away.
#define CLIFF_THRESHOLD_CM    40.0f   // calibrate for your mounting height
#define CLIFF_ALERT_INTERVAL  1000UL  // ms between idle CLIFF alerts

// Tilt angles visited at each horizontal position (up then back down)
const int TILT_ANGLES[]  = {60, 75, 90, 105, 120};
const int TILT_COUNT     = sizeof(TILT_ANGLES) / sizeof(TILT_ANGLES[0]);

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long encoderCount = 0;

void encoderISR() {
    // Rising edge on A: B HIGH = forward, B LOW = backward
    if (digitalRead(ENCODER_B)) encoderCount++;
    else                         encoderCount--;
}

// ── Servo object ──────────────────────────────────────────────────────────────
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

// ── Ultrasonic / cliff detection ──────────────────────────────────────────────
// Returns distance in cm; 999.0 if nothing detected within timeout.
float readUltrasonicDown() {
    digitalWrite(PIN_ULTRA_DOWN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRA_DOWN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRA_DOWN_TRIG, LOW);
    long us = pulseIn(PIN_ULTRA_DOWN_ECHO, HIGH, 30000UL);  // 30 ms → ~5 m max
    return (us == 0) ? 999.0f : us * 0.01715f;              // µs → cm
}

// Turn left (counter-clockwise) for a given number of degrees
void turnLeft(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, SWEEP_TURN_PWM, false);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, SWEEP_TURN_PWM, true);
    delay(ms);
    stopMotors();
}

// Turn right (clockwise) for a given number of degrees
void turnRight(float degrees) {
    int ms = (int)(degrees * MS_PER_DEGREE);
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, SWEEP_TURN_PWM, true);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, SWEEP_TURN_PWM, false);
    delay(ms);
    stopMotors();
}

// ── Sweep ─────────────────────────────────────────────────────────────────────
// Pause at one tilt angle, notify Pi, wait for NEXT or timeout
void tiltStep(float hAngle, int tiltAngle, int stepMs) {
    tiltServo.write(tiltAngle);
    Serial.print("AT:");
    Serial.print(hAngle);
    Serial.print(",");
    Serial.println(tiltAngle-MIDDLE_ANGLE);

    unsigned long deadline = millis() + stepMs;
    while (millis() < deadline) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line == "NEXT") return;
        }
    }
}

// range  — total horizontal span in degrees (sweeps from -range/2 to +range/2)
// stepMs — wait time at each tilt position for Pi to read LIDAR
void sweep(float range, int stepMs) {
    float halfRange = range / 2.0f;
    int   numSteps  = (int)(range / SWEEP_STEP_DEG);

    // Rotate to the left edge
    turnLeft(halfRange);

    for (int i = 0; i <= numSteps; i++) {
        float hAngle = -halfRange + i * SWEEP_STEP_DEG;

        // Sweep tilt up through all angles
        for (int t = 0; t < TILT_COUNT; t++) {
            tiltStep(hAngle, TILT_ANGLES[t], stepMs);
        }

        // Sweep tilt back down (skip the top angle — already visited)
        for (int t = TILT_COUNT - 2; t >= 0; t--) {
            tiltStep(hAngle, TILT_ANGLES[t], stepMs);
        }

        // Advance to next horizontal position (skip after last)
        if (i < numSteps) {
            turnRight(SWEEP_STEP_DEG);
        }
    }

    // Return tilt to centre and rotate back to heading 0
    tiltServo.write(90);
    turnLeft(halfRange);

    Serial.println("SWEEP_DONE");
}

// ── Handlers ──────────────────────────────────────────────────────────────────
void handleServo(int direction, float amount, float /*speed*/) {
    int angle = (int)(amount * 180.0f);
    angle = constrain(angle, 0, 180);

    switch (direction) {
        case DIR_UP:   tiltServo.write(angle); break;
        case DIR_DOWN: tiltServo.write(angle); break;
    }
}

void handleMotor(int direction, float distanceCm, float speed) {
    int pwm = (int)(speed * 255.0f);
    pwm = constrain(pwm, 0, 255);

    bool forward;
    switch (direction) {
        case DIR_UP:    forward = true;  break;
        case DIR_DOWN:  forward = false; break;
        case DIR_LEFT:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, false);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, true);
            return;
        case DIR_RIGHT:
            setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, true);
            setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, false);
            return;
        default:
            stopMotors();
            return;
    }

    // Start driving
    setMotor(PIN_ENA, PIN_IN1, PIN_IN2, pwm, forward);
    setMotor(PIN_ENB, PIN_IN3, PIN_IN4, pwm, forward);

    if (distanceCm <= 0.0f) {
        // Continuous mode — caller is responsible for stopping
        return;
    }

    // Distance-controlled mode: snapshot encoder, drive until target, then stop
    noInterrupts();
    long startCount = encoderCount;
    interrupts();

    long targetCounts = (long)(distanceCm / DIST_PER_REV_CM * COUNTS_PER_REV);
    unsigned long deadline = millis() + MOTOR_TIMEOUT_MS;
    bool cliffDetected = false;

    while (millis() < deadline) {
        noInterrupts();
        long travelled = abs(encoderCount - startCount);
        interrupts();
        if (travelled >= targetCounts) break;

        // Check downward ultrasonic — abort if ground disappears
        if (readUltrasonicDown() > CLIFF_THRESHOLD_CM) {
            cliffDetected = true;
            break;
        }
    }

    stopMotors();

    noInterrupts();
    long actualCounts = abs(encoderCount - startCount);
    interrupts();

    float actualCm = (float)actualCounts / COUNTS_PER_REV * DIST_PER_REV_CM;

    if (cliffDetected) {
        // Send partial distance covered; Pi will handle cliff response
        Serial.print("CLIFF:");
        Serial.println(actualCm);
        return;
    }

    // Normal completion — report distance and slip ratio
    // error > 1.0 → wheels spun more than expected (slipping)
    // error < 1.0 → fewer counts than expected (stalled or very slow)
    float errorRatio = (targetCounts > 0) ? (float)actualCounts / (float)targetCounts : 1.0f;
    Serial.print("DONE:");
    Serial.print(actualCm);
    Serial.print(",");
    Serial.println(errorRatio, 4);
}

void handleSweep(float amount, float speed) {
    sweep(amount, (int)speed);
}

void handleQuery() {
    // Atomically snapshot and reset the encoder counter
    noInterrupts();
    long count = encoderCount;
    encoderCount = 0;
    interrupts();

    float distanceCm = (float)count / COUNTS_PER_REV * DIST_PER_REV_CM;
    Serial.print("DIST:");
    Serial.println(distanceCm);
}

void dispatch(const Command& cmd) {
    switch (cmd.system) {
        case SYS_SERVO: handleServo(cmd.direction, cmd.amount, cmd.speed); break;
        case SYS_MOTOR: handleMotor(cmd.direction, cmd.amount, cmd.speed); break;
        case SYS_SWEEP: handleSweep(cmd.amount, cmd.speed);                break;
        case SYS_QUERY: handleQuery();                                      break;
    }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
unsigned long lastCliffAlert = 0;  // rate-limit idle cliff messages

void setup() {
    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    stopMotors();

    // Ultrasonic
    pinMode(PIN_ULTRA_DOWN_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_DOWN_ECHO, INPUT);

    // Encoder
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    tiltServo.attach(PIN_TILT);
    tiltServo.write(90);  // center

    // Handshake — resend READY every second until Pi replies ACK
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
    // Idle cliff check — rate-limited so Pi isn't flooded while stationary
    if (millis() - lastCliffAlert >= CLIFF_ALERT_INTERVAL) {
        float downDist = readUltrasonicDown();
        if (downDist > CLIFF_THRESHOLD_CM) {
            stopMotors();
            Serial.print("CLIFF:");
            Serial.println(downDist);
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

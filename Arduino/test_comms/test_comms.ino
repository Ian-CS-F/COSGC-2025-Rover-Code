/*
 * test_comms.ino — Arduino side of the Pi ↔ Arduino communication test.
 *
 * This sketch implements the same handshake and packet format as
 * rover_interpreeter.ino but replaces real hardware actions with immediate
 * echo responses so the Pi test script can verify every command type without
 * needing motors, servos, or sensors connected.
 *
 * Expected responses (Pi test script checks these):
 *   Servo  (0,_,amount,_)   → "SERVO_OK:<angle_deg>"
 *   Motor  (1,_,0,0)        → "MOTOR_OK"          (continuous / speed=0)
 *   Motor  (1,_,dist,_)     → "DONE:<dist>,1.0000" (distance > 0)
 *   Query  (3,0,0,0)        → "DIST:0.00"
 *   Flip   (4,0,1,0)        → "FLIP_OK:1"
 *   Flip   (4,0,0,0)        → "FLIP_OK:0"
 *   Malformed packet        → "PARSE_ERR"
 *
 * Handshake: Arduino sends "READY" every second until Pi replies "ACK".
 */

#define BAUD_RATE 9600

// System identifiers — must match rover_interpreeter.ino
#define SYS_SERVO 0
#define SYS_MOTOR 1
#define SYS_SWEEP 2
#define SYS_QUERY 3
#define SYS_FLIP  4

struct Command {
    int   system;
    int   direction;
    float amount;
    float speed;
};

// ── Packet parser (identical to rover_interpreeter.ino) ───────────────────────
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

// ── Command dispatch (echo only — no hardware) ────────────────────────────────
void dispatch(const Command& cmd) {
    switch (cmd.system) {

        case SYS_SERVO: {
            int angle = (int)(cmd.amount * 180.0f);
            angle = constrain(angle, 0, 180);
            Serial.print("SERVO_OK:");
            Serial.println(angle);
            break;
        }

        case SYS_MOTOR: {
            if (cmd.amount <= 0.0f) {
                // Continuous mode — just acknowledge
                Serial.println("MOTOR_OK");
            } else {
                // Distance-controlled mode — echo back the requested distance
                // with a perfect error ratio (simulated)
                Serial.print("DONE:");
                Serial.print(cmd.amount, 2);
                Serial.println(",1.0000");
            }
            break;
        }

        case SYS_SWEEP: {
            // Minimal sweep echo — one AT position then done
            Serial.println("AT:0,0");
            // Wait for Pi's NEXT
            unsigned long deadline = millis() + 3000UL;
            while (millis() < deadline) {
                if (Serial.available()) {
                    String line = Serial.readStringUntil('\n');
                    line.trim();
                    if (line == "NEXT") break;
                }
            }
            Serial.println("SWEEP_DONE");
            break;
        }

        case SYS_QUERY: {
            Serial.println("DIST:0.00");
            break;
        }

        case SYS_FLIP: {
            int state = (cmd.amount >= 0.5f) ? 1 : 0;
            Serial.print("FLIP_OK:");
            Serial.println(state);
            break;
        }

        default:
            Serial.println("UNKNOWN_SYS");
            break;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);

    // Handshake — send READY every second until Pi replies ACK
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
    // Blink the built-in LED to show we're ready
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (!Serial.available()) return;

    String raw = Serial.readStringUntil('\n');
    raw.trim();

    Command cmd;
    if (parsePacket(raw, cmd)) {
        dispatch(cmd);
    } else {
        Serial.println("PARSE_ERR");
    }
}

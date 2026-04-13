#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

static const int   ROTATE_SPEED = 150;
static const float HEADING_TOL  = 5.0f;

static percepetion perception;
static movement    motors(&perception);

static float heading      = 0.0f;
static float minUsDist    = 9999.0f;   // single minimum (used as threshold in RETURN)
static float minUsHeading = 0.0f;      // averaged heading of N closest readings
static unsigned long lastUs = 0;

// ── Top-N closest readings ────────────────────────────────────────────────────
static const int TOP_N = 10;
static float topDist[TOP_N];
static float topHead[TOP_N];
static int   topCount = 0;

static void insertTopN(float dist, float head) {
    // if not full, just append
    if (topCount < TOP_N) {
        topDist[topCount] = dist;
        topHead[topCount] = head;
        topCount++;
    } else {
        // find the largest dist in the list
        int worst = 0;
        for (int i = 1; i < TOP_N; i++) {
            if (topDist[i] > topDist[worst]) worst = i;
        }
        // replace it if new reading is closer
        if (dist < topDist[worst]) {
            topDist[worst] = dist;
            topHead[worst] = head;
        }
    }
}

static float avgTopHeading() {
    if (topCount == 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < topCount; i++) sum += topHead[i];
    return sum / topCount;
}

static float avgTopDist() {
    if (topCount == 0) return 9999.0f;
    float sum = 0.0f;
    for (int i = 0; i < topCount; i++) sum += topDist[i];
    return sum / topCount;
}
// ─────────────────────────────────────────────────────────────────────────────
static float lastValidUs  = -1.0f;      // last accepted US reading
static const float US_SPIKE_THRESHOLD = 60000.0f;  // cm — ignore jumps larger than this
static const int   US_WARMUP_READINGS = 5;       // discard first N readings
static int         usReadingCount     = 0;
static float       returnStartHeading = 0.0f;
static const float RETURN_MIN_DEG     = 20.0f;   // must rotate at least this far before stopping
static const int   RETURN_EXTRA_MS    = 100;      // extra ms to keep rotating after US condition met; tune this
static long        returnExtraStart   = -1;

static void updateHeading() {
    unsigned long now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;
    heading += perception.getGyroZ() * (180.0f / PI) * dt;
}

static const float APPROACH_STOP_CM  = 15.0f;
static const int   APPROACH_SPEED    = 200;
static const float FORWARD_STOP_MM   = 150.0f;  // stop forward when front IR < this (mm)
static const int   FORWARD_SPEED     = 200;

// ── US wall-follow PI (APPROACH_FORWARD) ─────────────────────────────────────
static const float WF_SETPOINT_CM = 15.0f;  // target distance from right wall (cm)
static const float WF_KP          = 15.0f;   // tune: higher = more aggressive correction
static const float WF_KI          = 0.5f;   // tune: reduces steady-state offset
static const float WF_MAX_INT     = 200.0f;

static float         wf_integral  = 0.0f;
static unsigned long wf_last_us   = 0;

static int wfCorrection() {
    unsigned long now = micros();
    float dt = (now - wf_last_us) / 1e6f;
    wf_last_us = now;
    if (dt > 0.1f) dt = 0.1f;

    float dist = perception.getUltrasonicCm();
    if (dist <= 0.0f) return 0;  // no echo — no correction

    float error = dist - WF_SETPOINT_CM;  // +ve = too far → strafe right (vy-)
    wf_integral += error * dt;
    wf_integral = constrain(wf_integral, -WF_MAX_INT, WF_MAX_INT);

    float vy = WF_KP * error + WF_KI * wf_integral;
    return (int)constrain(-vy, -300, 300);  // negate: vy+ = left, vy- = right
}
// ─────────────────────────────────────────────────────────────────────────────

enum State { SCAN, RETURN, APPROACH, APPROACH_FORWARD, DONE };
static State state = SCAN;

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);
    // flush gyro drift before starting — let it settle for 500ms
    unsigned long settle = millis();
    while (millis() - settle < 500) {
        perception.update();
    }
    heading   = 0.0f;
    lastUs    = micros();
    Serial.println("START: spinning 360");
}

void loop() {
    perception.update();

    switch (state) {
    case SCAN:
        updateHeading();
        {
            static unsigned long lastSampleMs = 0;
            unsigned long now = millis();
            if (now - lastSampleMs >= 100) {
                lastSampleMs = now;
                float usDist = perception.getUltrasonicCm();
                usReadingCount++;
                bool valid = (usReadingCount > US_WARMUP_READINGS) &&
                             (usDist > 0.0f) &&
                             (lastValidUs < 0.0f || fabsf(usDist - lastValidUs) <= US_SPIKE_THRESHOLD);
                if (valid) {
                    lastValidUs = usDist;
                    insertTopN(usDist, heading);
                    if (usDist < minUsDist) minUsDist = usDist;
                }
            }
        }
        motors.RotateCCW(ROTATE_SPEED);
        if (heading >= 360.0f) {
            motors.Stop(true);
            lastUs = micros();
            returnStartHeading = heading;
            minUsHeading = avgTopHeading();
            float avgDist = avgTopDist();
            Serial.print("Scan done. Avg closest dist: ");
            Serial.print(avgDist, 1);
            Serial.print(" cm at avg heading ");
            Serial.println(minUsHeading, 1);
            state = RETURN;
        }
        break;

    case RETURN:
        updateHeading();
        {
            float rotated = returnStartHeading - heading;  // CW so heading decreases
            float usDist  = perception.getUltrasonicCm();
            if (rotated >= RETURN_MIN_DEG && usDist > 0.0f && usDist <= minUsDist + 2.0f) {
                if (returnExtraStart < 0) returnExtraStart = millis();
                if (millis() - returnExtraStart >= RETURN_EXTRA_MS) {
                    motors.Stop(true);
                    Serial.print("Facing closest wall. US=");
                    Serial.print(usDist, 1);
                    Serial.println(" — moving right");
                    state = APPROACH;
                } else {
                    motors.RotateCW(150);
                }
            } else {
                motors.RotateCW(150);
            }
        }
        break;

    case APPROACH:
        {
            float usDist = perception.getUltrasonicCm();
            if (usDist > 0.0f && usDist < APPROACH_STOP_CM) {
                motors.Stop(true);
                Serial.print("At side wall. US=");
                Serial.print(usDist, 1);
                Serial.println(" — moving forward");
                state = APPROACH_FORWARD;
            } else {
                motors.MoveRight(APPROACH_SPEED);
            }
        }
        break;

    case APPROACH_FORWARD:
        {
            float frontMm = perception.getIRMedFront();
            if (frontMm > 0.0f && frontMm < FORWARD_STOP_MM) {
                motors.Stop(true);
                Serial.print("At front wall. IR=");
                Serial.println(frontMm, 1);
                state = DONE;
            } else {
                int vy = wfCorrection();
                motors.drive(FORWARD_SPEED, vy, (int)motors.headingCorrection());
            }
        }
        break;

    case DONE:
        motors.Stop(true);
        break;
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        const char* stateStr = state == SCAN ? "SCAN" : state == RETURN ? "RETURN" : state == APPROACH ? "APPROACH" : state == APPROACH_FORWARD ? "APPROACH_FWD" : "DONE";
        Serial.print(stateStr);
        Serial.print("  heading=");
        Serial.print(heading, 1);
        Serial.print("  us=");
        Serial.println(perception.getUltrasonicCm(), 1);
    }
}

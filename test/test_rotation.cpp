#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   ROTATE_SPEED  = 150;    // motor speed 0–1000
static const float TARGET_DEG    = 360.0f; // degrees to rotate
static const float HEADING_TOL   = 5.0f;   // stop within ±this many degrees
static const int   SETTLE_MS     = 500;    // gyro settle time before starting
// ─────────────────────────────────────────────────────────────────────────────

static percepetion perception;
static movement    motors(&perception);

static float          heading  = 0.0f;
static unsigned long  lastUs   = 0;

static void updateHeading() {
    unsigned long now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;
    heading += perception.getGyroZ() * (180.0f / PI) * dt;
}

enum State { SPINNING, DONE };
static State state = SPINNING;

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();

    // settle gyro
    delay(2000);
    unsigned long settle = millis();
    while (millis() - settle < SETTLE_MS) {
        perception.update();
    }
    heading = 0.0f;
    lastUs  = micros();

    Serial.println("# heading_deg,state");
    Serial.println("START");
}

void loop() {
    perception.update();

    if (state == SPINNING) {
        updateHeading();
        if (heading >= TARGET_DEG - HEADING_TOL) {
            motors.Stop(true);
            state = DONE;
            Serial.print("DONE  final_heading=");
            Serial.println(heading, 1);
        } else {
            motors.RotateCCW(ROTATE_SPEED);
        }
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print(heading, 1);
        Serial.println(state == SPINNING ? ",SPINNING" : ",DONE");
    }
}

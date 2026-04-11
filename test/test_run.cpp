#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   MOVE_SPEED        = 200;
static const float REAR_STOP_MM      = 150.0f;   // stop moving down  when rear IR  <= this
static const float FRONT_STOP_MM     = 150.0f;   // stop moving up    when front IR <= this
static const float LEFT_STOP_MM      = 120.0f;   // left wall threshold (mm)
static const float LEFT_IGNORE_MM    = 110.0f;   // ignore readings at/below this (default no-detect value)
static const int   LEFT_CONFIRM_N    = 20;  // non-100 readings needed to confirm wall
static const int   LEFT_RESET_N      = 10;  // 100mm readings needed to reset
static const unsigned long STRAFE_TIME_MS = 800;

static int           leftNonIgnoreCount = 0;
static int           leftIgnoreCount    = 0;
static unsigned long leftLastCountedMs  = 0;
// ─────────────────────────────────────────────────────────────────────────────

static percepetion perception;
static movement    motors(&perception);

enum State {
    MOVE_DOWN,          // move backward until rear IR <= REAR_STOP_MM
    STRAFE_LEFT_A,      // strafe left for STRAFE_TIME_MS
    MOVE_UP,            // move forward until front IR <= FRONT_STOP_MM
    STRAFE_LEFT_B,      // strafe left for STRAFE_TIME_MS
    FINAL_MOVE_DOWN,    // final move down (left wall detected during MOVE_DOWN)
    FINAL_MOVE_UP,      // final move up   (left wall detected during MOVE_UP)
    DONE
};

static State         state          = MOVE_DOWN;
static unsigned long strafeStart    = 0;

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);
    Serial.println("START");
}

static bool leftWallSeen    = false;  // set after 20 non-100 readings
static bool leftWallDetected() {
    float leftMm = perception.getIRLongLeft();
    unsigned long now = millis();
    if (now - leftLastCountedMs < 50) return false;
    leftLastCountedMs = now;

    if (!leftWallSeen) {
        // phase 1: accumulate 20 non-100 readings
        if (leftMm <= LEFT_IGNORE_MM) {
            leftNonIgnoreCount = 0;  // reset on any 100mm reading
        } else {
            leftNonIgnoreCount++;
            if (leftNonIgnoreCount >= LEFT_CONFIRM_N) {
                leftWallSeen       = true;
                leftIgnoreCount    = 0;
                Serial.println("Left wall seen — waiting for 10x 100mm");
            }
        }
        return false;
    } else {
        // phase 2: now wait for 10 consecutive 100mm readings
        if (leftMm <= LEFT_IGNORE_MM) {
            leftIgnoreCount++;
        } else {
            leftIgnoreCount = 0;  // reset if we get a non-100 again
        }
        return leftIgnoreCount >= LEFT_RESET_N;
    }
}

void loop() {
    perception.update();

    switch (state) {

    case MOVE_DOWN:
        if (leftWallDetected()) {
            motors.Stop(true);
            Serial.println("Left wall detected — final move down");
            state = FINAL_MOVE_DOWN;
            break;
        }
        {
            float rearMm = perception.getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors.Stop(true);
                Serial.println("Rear wall — strafe left");
                strafeStart = millis();
                state = STRAFE_LEFT_A;
            } else {
                motors.MoveBackward(MOVE_SPEED);
            }
        }
        break;

    case STRAFE_LEFT_A:
        if (millis() - strafeStart >= STRAFE_TIME_MS) {
            motors.Stop(true);
            Serial.println("Strafe done — move up");
            state = MOVE_UP;
        } else {
            motors.MoveLeft(MOVE_SPEED);
        }
        break;

    case MOVE_UP:
        if (leftWallDetected()) {
            motors.Stop(true);
            Serial.println("Left wall detected — final move up");
            state = FINAL_MOVE_UP;
            break;
        }
        {
            float frontMm = perception.getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors.Stop(true);
                Serial.println("Front wall — strafe left");
                strafeStart = millis();
                state = STRAFE_LEFT_B;
            } else {
                motors.MoveForward(MOVE_SPEED);
            }
        }
        break;

    case STRAFE_LEFT_B:
        if (millis() - strafeStart >= STRAFE_TIME_MS) {
            motors.Stop(true);
            Serial.println("Strafe done — move down");
            state = MOVE_DOWN;
        } else {
            motors.MoveLeft(MOVE_SPEED);
        }
        break;

    case FINAL_MOVE_DOWN:
        {
            float rearMm = perception.getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors.Stop(true);
                Serial.println("DONE");
                state = DONE;
            } else {
                motors.MoveBackward(MOVE_SPEED);
            }
        }
        break;

    case FINAL_MOVE_UP:
        {
            float frontMm = perception.getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors.Stop(true);
                Serial.println("DONE");
                state = DONE;
            } else {
                motors.MoveForward(MOVE_SPEED);
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
        const char* s = state == MOVE_DOWN       ? "MOVE_DOWN"    :
                        state == STRAFE_LEFT_A   ? "STRAFE_A"     :
                        state == MOVE_UP         ? "MOVE_UP"      :
                        state == STRAFE_LEFT_B   ? "STRAFE_B"     :
                        state == FINAL_MOVE_DOWN ? "FINAL_DOWN"   :
                        state == FINAL_MOVE_UP   ? "FINAL_UP"     : "DONE";
        Serial.print(s);
        Serial.print("  front="); Serial.print(perception.getIRMedFront(), 0);
        Serial.print("  rear=");  Serial.print(perception.getIRLongRear(), 0);
        Serial.print("  left=");  Serial.println(perception.getIRLongLeft(), 0);
    }
}

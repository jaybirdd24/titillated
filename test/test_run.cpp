#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   MOVE_SPEED        = 200;
static const float REAR_STOP_MM      = 150.0f;
static const float FRONT_STOP_MM     = 150.0f;
static const float LEFT_IGNORE_MM    = 110.0f;  // below this = no left wall
static const int   LEFT_CONFIRM_N    = 20;
static const int   LEFT_RESET_N      = 10;

// ── Wall-follow PI (right IR, incrementing setpoint) ─────────────────────────
static const float WF_START_MM       = 100.0f;  // initial setpoint: 10 cm
static const float WF_STRAFE_INC_MM  = 100.0f;  // increment 10 cm per strafe
static const float WF_KP             =   1.0f;
static const float WF_KI             =   0.001f;
static const float WF_MAX_INT        = 200.0f;

static float         wf_setpoint_mm = WF_START_MM;
static float         wf_integral    = 0.0f;
static unsigned long wf_last_us     = 0;
// ─────────────────────────────────────────────────────────────────────────────

static percepetion perception;
static movement    motors(&perception);

// +vy = strafe left, -vy = strafe right
static int rightWfCorrection() {
    unsigned long now = micros();
    float dt = constrain((now - wf_last_us) / 1e6f, 0.0f, 0.1f);
    wf_last_us = now;
    float dist = perception.getUltrasonicCm() * 10.0f;  // cm → mm
    if (dist <= 0.0f) return 0;
    float error = dist - wf_setpoint_mm;
    wf_integral = constrain(wf_integral + error * dt, -WF_MAX_INT, WF_MAX_INT);
    return (int)constrain(-(WF_KP * error + WF_KI * wf_integral), -300, 300);
}

static void resetWf() { wf_integral = 0.0f; wf_last_us = micros(); }

enum State {
    MOVE_DOWN,        // move backward until rear IR <= REAR_STOP_MM
    STRAFE_LEFT_A,    // strafe left until right IR reaches next setpoint
    MOVE_UP,          // move forward until front IR <= FRONT_STOP_MM
    STRAFE_LEFT_B,    // strafe left until right IR reaches next setpoint
    FINAL_MOVE_DOWN,  // final move down (left wall detected during MOVE_DOWN)
    FINAL_MOVE_UP,    // final move up   (left wall detected during MOVE_UP)
    DONE
};

static State state = MOVE_DOWN;

// ── Left wall detection ───────────────────────────────────────────────────────
static bool          leftWallSeen       = false;
static int           leftNonIgnoreCount = 0;
static int           leftIgnoreCount    = 0;
static unsigned long leftLastCountedMs  = 0;

static bool leftWallDetected() {
    float leftMm = perception.getIRLongLeft();
    unsigned long now = millis();
    if (now - leftLastCountedMs < 50) return false;
    leftLastCountedMs = now;

    if (!leftWallSeen) {
        if (leftMm <= LEFT_IGNORE_MM) {
            leftNonIgnoreCount = 0;
        } else {
            leftNonIgnoreCount++;
            if (leftNonIgnoreCount >= LEFT_CONFIRM_N) {
                leftWallSeen    = true;
                leftIgnoreCount = 0;
                Serial.println("Left wall seen");
            }
        }
        return false;
    } else {
        if (leftMm <= LEFT_IGNORE_MM) leftIgnoreCount++;
        else                          leftIgnoreCount = 0;
        return leftIgnoreCount >= LEFT_RESET_N;
    }
}
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);
    resetWf();
    Serial.println("START");
}

void loop() {
    perception.update();

    switch (state) {

    case MOVE_DOWN:
        if (leftWallDetected()) {
            motors.Stop(true);
            Serial.println("Left wall — final move down");
            resetWf();
            state = FINAL_MOVE_DOWN;
            break;
        }
        {
            float rearMm = perception.getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors.Stop(true);
                Serial.print("Rear wall — strafing to ");
                Serial.print(wf_setpoint_mm + WF_STRAFE_INC_MM, 0);
                Serial.println(" mm");
                resetWf();
                state = STRAFE_LEFT_A;
            } else {
                motors.drive(-MOVE_SPEED, rightWfCorrection(), (int)motors.headingCorrection());
            }
        }
        break;

    case STRAFE_LEFT_A:
        {
            float rightMm = perception.getUltrasonicCm() * 10.0f;
            if (rightMm > 0.0f && rightMm >= wf_setpoint_mm + WF_STRAFE_INC_MM) {
                motors.Stop(true);
                wf_setpoint_mm += WF_STRAFE_INC_MM;
                resetWf();
                Serial.print("Strafe done — setpoint now ");
                Serial.print(wf_setpoint_mm, 0);
                Serial.println(" mm — move up");
                state = MOVE_UP;
            } else {
                motors.MoveLeft(MOVE_SPEED);
            }
        }
        break;

    case MOVE_UP:
        if (leftWallDetected()) {
            motors.Stop(true);
            Serial.println("Left wall — final move up");
            resetWf();
            state = FINAL_MOVE_UP;
            break;
        }
        {
            float frontMm = perception.getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors.Stop(true);
                Serial.print("Front wall — strafing to ");
                Serial.print(wf_setpoint_mm + WF_STRAFE_INC_MM, 0);
                Serial.println(" mm");
                resetWf();
                state = STRAFE_LEFT_B;
            } else {
                motors.drive(MOVE_SPEED, rightWfCorrection(), (int)motors.headingCorrection());
            }
        }
        break;

    case STRAFE_LEFT_B:
        {
            float rightMm = perception.getUltrasonicCm() * 10.0f;
            if (rightMm > 0.0f && rightMm >= wf_setpoint_mm + WF_STRAFE_INC_MM) {
                motors.Stop(true);
                wf_setpoint_mm += WF_STRAFE_INC_MM;
                resetWf();
                Serial.print("Strafe done — setpoint now ");
                Serial.print(wf_setpoint_mm, 0);
                Serial.println(" mm — move down");
                state = MOVE_DOWN;
            } else {
                motors.MoveLeft(MOVE_SPEED);
            }
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
                motors.drive(-MOVE_SPEED, rightWfCorrection(), (int)motors.headingCorrection());
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
                motors.drive(MOVE_SPEED, rightWfCorrection(), (int)motors.headingCorrection());
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
        const char* s = state == MOVE_DOWN       ? "MOVE_DOWN"  :
                        state == STRAFE_LEFT_A   ? "STRAFE_A"   :
                        state == MOVE_UP         ? "MOVE_UP"    :
                        state == STRAFE_LEFT_B   ? "STRAFE_B"   :
                        state == FINAL_MOVE_DOWN ? "FINAL_DOWN" :
                        state == FINAL_MOVE_UP   ? "FINAL_UP"   : "DONE";
        Serial.print(s);
        Serial.print("  sp=");  Serial.print(wf_setpoint_mm, 0);
        Serial.print("  us=");  Serial.print(perception.getUltrasonicCm() * 10.0f, 0);
        Serial.print("  rear=");  Serial.print(perception.getIRLongRear(), 0);
        Serial.print("  front="); Serial.print(perception.getIRMedFront(), 0);
        Serial.print("  left=");  Serial.println(perception.getIRLongLeft(), 0);
    }
}

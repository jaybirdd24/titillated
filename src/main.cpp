#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine;

// ── IK + Wall-Follow Test ────────────────────────────────────────────────────
// Runs a timed sequence so you can verify each channel independently.
// Place the robot on the floor with ~150 mm clearance on its LEFT side.
//
// Stage 0 (3 s)  — stationary, prints sensor baseline
// Stage 1 (2 s)  — drive(300, 0, 0)     forward only, no correction
// Stage 2 (2 s)  — drive(0, 300, 0)     strafe LEFT only
// Stage 3 (2 s)  — drive(0, -300, 0)    strafe RIGHT only
// Stage 4 (3 s)  — drive(300, vy, wz)   full 3-PID wall-follow forward
// Stage 5        — stop forever, keep printing sensor readings
// ─────────────────────────────────────────────────────────────────────────────

static const int   TEST_SPEED      = 300;    // 0–1000
static const float WALL_SETPOINT   = 150.0f; // mm from left wall

void setup()
{
    Serial.begin(9600);
    perception.init();
    motors.enable();

    Serial.println("=== IK TEST START ===");
    Serial.println("stage,left_ir_mm,vy_pid,wz_pid");
}

void loop()
{
    perception.update();

    unsigned long t = millis();
    int stage;

    if      (t <  3000) stage = 0;
    else if (t <  5000) stage = 1;
    else if (t <  7000) stage = 2;
    else if (t <  9000) stage = 3;
    else if (t < 12000) stage = 4;
    else                stage = 5;

    float left_mm = perception.getIRLongLeft();
    float wz      = motors.headingCorrection();
    float vy      = motors.wallFollowCorrection(WALL_SETPOINT);

    switch (stage) {
        case 0: motors.Stop(true);                          break;
        case 1: motors.drive(TEST_SPEED,  0,  0);           break; // forward
        case 2: motors.drive(0,  TEST_SPEED,  0);           break; // strafe left
        case 3: motors.drive(0, -TEST_SPEED,  0);           break; // strafe right
        case 4: motors.drive(TEST_SPEED, (int)vy, (int)wz); break; // full 3-PID
        case 5: motors.Stop(true);                          break;
    }

    // Print CSV at ~10 Hz
    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        Serial.print(stage);
        Serial.print(",");
        Serial.print(left_mm, 1);
        Serial.print(",");
        Serial.print(vy, 2);
        Serial.print(",");
        Serial.println(wz, 2);
    }
}

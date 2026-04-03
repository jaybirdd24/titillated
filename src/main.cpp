#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

percepetion perception;
movement    motors(&perception);

static const int   FORWARD_SPEED      = 300;     // 0–1000
static const float WALL_SETPOINT     = 300.0f;  // mm from left wall
static const float CRASH_THRESHOLD   = 105.0f;  // mm — emergency stop distance

void setup()
{
    Serial.begin(9600);
    perception.init();
    motors.enable();

    // Let sensors settle, then lock in the current heading
    delay(1000);
    perception.update();
    motors.latchHeading();

    Serial.println("=== WALL FOLLOW START ===");
    Serial.println("=== Distances (mm) ===");
}

void loop()
{
    perception.update();

    // Emergency stop if any sensor reads below 100 mm
    if (perception.isObstacleTooClose(CRASH_THRESHOLD)) {
        motors.Stop(true);
        static unsigned long last_warn = 0;
        if (millis() - last_warn >= 500) {
            last_warn = millis();
            // Serial.println("!! OBSTACLE TOO CLOSE — STOPPED !!");
        }
    }

    float front_mm = perception.getIRMedFront();
    float left_mm  = perception.getIRLongLeft();
    float right_mm = perception.getIRMedRight();
    float rear_mm  = perception.getIRLongRear();
    float wz       = motors.headingCorrection();
    float vy       = motors.wallFollowCorrection(WALL_SETPOINT);

    motors.drive(FORWARD_SPEED, (int)vy, (int)wz);

    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        Serial.print(F("Front:")); Serial.print(front_mm, 1); Serial.print(F("  "));
        Serial.print(F("Left:"));  Serial.print(left_mm,  1); Serial.print(F("  "));
        Serial.print(F("Right:")); Serial.print(right_mm, 1); Serial.print(F("  "));
        Serial.print(F("Rear:"));  Serial.println(rear_mm, 1);
    }
}

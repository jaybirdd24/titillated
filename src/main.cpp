#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine(&perception, &motors);

static const char* stateName(HomingState s)
{
    switch (s)
    {
        case HOMING_IDLE:           return "IDLE";
        case HOMING_ROTATE_SCAN:    return "ROTATE_SCAN";
        case HOMING_ROTATE_TO_MIN:  return "ROTATE_TO_MIN";
        case HOMING_APPROACH_WALL:  return "APPROACH_WALL";
        case HOMING_APPROACH_SIDE:  return "APPROACH_SIDE";
        case HOMING_DONE:           return "DONE";
        default:                    return "UNKNOWN";
    }
}

void setup()
{
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(1000);
    perception.update();
    Serial.println("=== TEST: Scan + Approach + Side Scan ===");
    Serial.println("State,Heading_deg,US_cm");
}

void loop()
{
    perception.update();

    stateMachine.fsmUpdate();

    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        Serial.print(stateName(stateMachine.getState()));
        Serial.print(",");
        Serial.print(stateMachine.getHeading(), 2);
        Serial.print(",");
        Serial.println(perception.getUltrasonicCm(), 1);
    }
}

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
        case HOMING_IDLE:             return "IDLE";
        case HOMING_ROTATE_SCAN:      return "ROTATE_SCAN";
        case HOMING_ROTATE_TO_MIN:    return "ROTATE_TO_MIN";
        case HOMING_APPROACH_WALL:    return "APPROACH_WALL";
        case HOMING_SCAN_SIDE_RIGHT:  return "SCAN_SIDE_RIGHT";
        case HOMING_SCAN_SIDE_LEFT:   return "SCAN_SIDE_LEFT";
        case HOMING_ROTATE_TO_SIDE:   return "ROTATE_TO_SIDE";
        case HOMING_APPROACH_SIDE:    return "APPROACH_SIDE";
        case HOMING_LINEUP:           return "LINEUP";
        case HOMING_DONE:             return "DONE";
        default:                      return "UNKNOWN";
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

    HomingState state = stateMachine.getState();

    // Run through: IDLE -> ROTATE_SCAN -> ROTATE_TO_MIN -> APPROACH_WALL
    //              -> SCAN_SIDE_RIGHT -> SCAN_SIDE_LEFT -> ROTATE_TO_SIDE
    // Stop once facing the closer side wall (APPROACH_SIDE reached).
    if (state == HOMING_ROTATE_TO_MIN) {
        motors.Stop(true);
    } else {
        stateMachine.fsmUpdate();
    }

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

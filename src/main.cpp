#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine(&perception, &motors);

static const char* stateName(RobotState s) {
    switch (s) {
        case HOMING_IDLE:          return "HOMING_IDLE";
        case HOMING_SCAN:          return "HOMING_SCAN";
        case HOMING_ANALYSE:       return "HOMING_ANALYSE";
        case HOMING_RETURN:        return "HOMING_RETURN";
        case HOMING_APPROACH_WALL: return "HOMING_APPROACH_WALL";
        case HOMING_APPROACH_FWD:  return "HOMING_APPROACH_FWD";
        case HOMING_RAM_WALL:      return "HOMING_RAM_WALL";
        case HOMING_BACK_OFF:      return "HOMING_BACK_OFF";
        case RUN_MOVE_DOWN:        return "RUN_MOVE_DOWN";
        case RUN_STRAFE_LEFT_A:    return "RUN_STRAFE_A";
        case RUN_MOVE_UP:          return "RUN_MOVE_UP";
        case RUN_STRAFE_LEFT_B:    return "RUN_STRAFE_B";
        case RUN_FINAL_MOVE_DOWN:  return "RUN_FINAL_DOWN";
        case RUN_FINAL_MOVE_UP:    return "RUN_FINAL_UP";
        case STATE_DONE:           return "DONE";
        default:                   return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);

    // settle gyro
    unsigned long settle = millis();
    while (millis() - settle < 500) {
        perception.update();
    }

    Serial.println("=== START ===");
    Serial.println("State,Heading,US_cm");
}

void loop() {
    perception.update();
    stateMachine.fsmUpdate();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        // Serial.print(stateName(stateMachine.getState()));
        // Serial.print(",");
        // Serial.print(stateMachine.getHeading(), 1);
        // Serial.print(",");
        // Serial.println(perception.getUltrasonicCm(), 1);
        float length = perception.getIRLongLeft();
        Serial.print("IR long left (mm): ");
        Serial.println(length, 1);
    }
}

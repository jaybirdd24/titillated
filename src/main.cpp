#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"
#include "coordinates.h"
#include "comms.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine(&perception, &motors);
coordinates coords(&stateMachine, &motors, &perception);
comms       wireless;

static const char* stateName(RobotState s) {
    switch (s) {
        case HOMING_IDLE:          return "HOMING_IDLE";
        case HOMING_SCAN:          return "HOMING_SCAN";
        case HOMING_RETURN:        return "HOMING_RETURN";
        case HOMING_APPROACH_WALL: return "HOMING_APPROACH_WALL";
        case HOMING_APPROACH_FWD:  return "HOMING_APPROACH_FWD";
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

    wireless.init();

    Serial.println("=== START ===");
    Serial.println("State,Heading,US_cm,X_mm,Y_mm");
}

void loop() {
    perception.update();
    stateMachine.fsmUpdate();
    coords.update();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();

        // USB serial telemetry
        Serial.print(stateName(stateMachine.getState()));
        Serial.print(",");
        Serial.print(stateMachine.getHeading(), 1);
        Serial.print(",");
        Serial.print(perception.getUltrasonicCm(), 1);
        Serial.print(",");
        Serial.print(coords.getX(), 1);
        Serial.print(",");
        Serial.println(coords.getY(), 1);

        // Wireless CSV logging
        wireless.sendCSV(millis(), coords.getX(), coords.getY(), coords.getYaw());
    }
}

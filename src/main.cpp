#include <Arduino.h>
#include <SoftwareSerial.h>
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

static SoftwareSerial WirelessSerial(10, 11);  // RX, TX (HC-12 module)

void setup() {
    Serial.begin(115200);
    WirelessSerial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);

    // settle gyro
    unsigned long settle = millis();
    while (millis() - settle < 500) {
        perception.update();
    }

    Serial.println("=== START ===");
    WirelessSerial.println("=== START ===");
    // CSV header
    const char* hdr = "ms,state,heading,us_cm,target_hdg,target_dist_cm,ir_front_mm,ir_right_mm,ir_rear_mm,ir_left_mm";
    Serial.println(hdr);
    WirelessSerial.println(hdr);
}

void loop() {
    perception.update();
    stateMachine.fsmUpdate();

    static unsigned long lastPrint = 0;

    RobotState s = stateMachine.getState();

    if (s <= HOMING_BACK_OFF) {
        unsigned long now = millis();
        if (now - lastPrint >= 20) {   // 20 Hz
            lastPrint = now;

            float heading = stateMachine.getHeading();
            float us      = perception.getUltrasonicCm();
            float tgtHdg  = stateMachine.getTargetHeading();
            float tgtDist = stateMachine.getTargetDistCm();
            float irFront = perception.getIRMedFront();
            float irRight = perception.getIRMedRight();
            float irRear  = perception.getIRLongRear();
            float irLeft  = perception.getIRLongLeft();

            // helper lambda to print one CSV row to a Stream
            auto printRow = [&](Stream &out) {
                out.print(now);          out.print(',');
                out.print(stateName(s)); out.print(',');
                out.print(heading, 1);   out.print(',');
                out.print(us, 1);        out.print(',');
                out.print(tgtHdg, 1);    out.print(',');
                out.print(tgtDist, 1);   out.print(',');
                out.print(irFront, 1);   out.print(',');
                out.print(irRight, 1);   out.print(',');
                out.print(irRear, 1);    out.print(',');
                out.println(irLeft, 1);
            };

            printRow(Serial);
            printRow(WirelessSerial);
        }
    }
}

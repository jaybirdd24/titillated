#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"
#include "comms.h"

SoftwareSerial WirelessSerial(10, 11);  // RX, TX — change pins here

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
    WirelessSerial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);

    // settle gyro
    unsigned long settle = millis();
    while (millis() - settle < 500) {
        perception.update();
    }

    WirelessSerial.println("=== START ===");
    // CSV header
    WirelessSerial.println(
        "t_ms,state_id,state,heading_deg,target_hdg_deg,"
        "gyro_z_rads,vx_cmd,vy_cmd,wz_cmd,"
        "ir_front_mm,ir_rear_mm,ir_left_mm,ir_right_mm,us_cm"
    );
}

void loop() {
    perception.update();

    // Detect state transitions and emit EVT lines
    static RobotState prevState = HOMING_IDLE;
    RobotState curState = stateMachine.getState();
    if (curState != prevState) {
        WirelessSerial.print("EVT:");
        WirelessSerial.print(millis());
        WirelessSerial.print(" state_enter ");
        WirelessSerial.println(stateName(curState));
        prevState = curState;
    }

    stateMachine.fsmUpdate();

    // 20 Hz structured telemetry
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();

        WirelessSerial.print(millis());               WirelessSerial.print(',');
        WirelessSerial.print((int)curState);          WirelessSerial.print(',');
        WirelessSerial.print(stateName(curState));    WirelessSerial.print(',');
        WirelessSerial.print(motors.getHeading(), 2); WirelessSerial.print(',');
        WirelessSerial.print(motors.getTargetHeading(), 2); WirelessSerial.print(',');
        WirelessSerial.print(perception.getGyroZ(), 4); WirelessSerial.print(',');
        WirelessSerial.print(motors.getLastVx());     WirelessSerial.print(',');
        WirelessSerial.print(motors.getLastVy());     WirelessSerial.print(',');
        WirelessSerial.print(motors.getLastWz());     WirelessSerial.print(',');
        WirelessSerial.print(perception.getIRMedFront(), 1);  WirelessSerial.print(',');
        WirelessSerial.print(perception.getIRLongRear(), 1);  WirelessSerial.print(',');
        WirelessSerial.print(perception.getIRLongLeft(), 1);  WirelessSerial.print(',');
        WirelessSerial.print(perception.getIRMedRight(), 1);  WirelessSerial.print(',');
        WirelessSerial.println(perception.getUltrasonicCm(), 2);
    }
}

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine(&perception, &motors);

// HC-12 wireless logger
static SoftwareSerial WirelessSerial(10, 11);   // RX, TX

// Logging configuration
static const unsigned long LOG_INTERVAL_MS = 150;   // 10 Hz
static unsigned long lastLogMs = 0;
static uint32_t logSeq = 0;
static RobotState lastLoggedState = HOMING_IDLE;
static bool firstStateLog = true;

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
        case RUN_STRAFE_LEFT_A:    return "RUN_STRAFE_LEFT_A";
        case RUN_MOVE_UP:          return "RUN_MOVE_UP";
        case RUN_STRAFE_LEFT_B:    return "RUN_STRAFE_LEFT_B";
        case RUN_FINAL_MOVE_DOWN:  return "RUN_FINAL_MOVE_DOWN";
        case RUN_FINAL_MOVE_UP:    return "RUN_FINAL_MOVE_UP";
        case STATE_DONE:           return "STATE_DONE";
        default:                   return "UNKNOWN";
    }
}

static void printFloatOrNA(Stream &out, float v, int dp = 2) {
    if (isnan(v)) {
        out.print("nan");
        return;
    }
    out.print(v, dp);
}

static void logHeader(Stream &out) {
    out.println(
        "t_ms,seq,type,state,heading_deg,gyroZ_rads,us_cm,"
        "ir_long_rear_mm,ir_med_front_mm,ir_med_right_mm"
    );
}

static void logRow(Stream &out, const char *typeTag) {
    out.print(millis());
    out.print(",");

    // out.print(logSeq++);
    // out.print(",");

    // out.print(typeTag);
    // out.print(",");

    out.print(stateName(stateMachine.getState()));
    out.print(",");

    // printFloatOrNA(out, stateMachine.getHeading(), 2);
    // out.print(",");

    // printFloatOrNA(out, perception.getGyroZ(), 4);
    // out.print(",");

    printFloatOrNA(out, perception.getUltrasonicCm(), 2);
    out.print(",");

    printFloatOrNA(out, perception.getIRLongRear(), 1);
    out.print(",");

    printFloatOrNA(out, perception.getIRMedFront(), 1);
    out.print(",");

    printFloatOrNA(out, perception.getIRMedRight(), 1);
    out.println();
}

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
    logHeader(WirelessSerial);

    // force first state event
    lastLoggedState = stateMachine.getState();
    firstStateLog = true;
}

void loop() {
    perception.update();
    stateMachine.fsmUpdate();

    RobotState currentState = stateMachine.getState();

    // Log state transitions immediately
    if (firstStateLog || currentState != lastLoggedState) {
        lastLoggedState = currentState;
        firstStateLog = false;
        logRow(WirelessSerial, "EVENT");
    }

    // Periodic telemetry log
    unsigned long now = millis();
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = now;
        logRow(WirelessSerial, "DATA");
    }
}
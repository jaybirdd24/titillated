#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"
#include "pose_estimator.h"
#include "robot_config.h"

percepetion perception;
pose_estimator estimator;
movement    motors(&perception, &estimator);
fsm         stateMachine(&perception, &motors, &estimator);

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
    estimator.begin();
    motors.enable();
    delay(2000);

    // settle gyro
    unsigned long settle = millis();
    while (millis() - settle < 500) {
        perception.update();
    }
    perception.calibrateGyro();
    motors.latchHeading();

    Serial.println("=== START ===");
    Serial.println("State,x_mm,y_mm,theta_deg,front_mm,rear_mm,left_mm,right_mm,us_mm");
}

void loop() {
    perception.update();

    static unsigned long lastPredictUs = 0;
    unsigned long nowUs = micros();
    float dt = 0.0f;
    if (lastPredictUs != 0) {
        dt = (nowUs - lastPredictUs) / 1e6f;
    }
    lastPredictUs = nowUs;

    SensorReadings readings = {
        perception.getIRMedFront(),
        perception.getIRLongRear(),
        perception.getIRLongLeft(),
        perception.getIRMedRight(),
        perception.getUltrasonicMm(),
        perception.getGyroZ()
    };

    estimator.predict(motors.getCurrentMotionCommand(), readings.gyro_z_rad_s, dt);
    estimator.correct(readings);
    stateMachine.fsmUpdate();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Pose2D pose = stateMachine.getPose();
        Serial.print(stateName(stateMachine.getState()));
        Serial.print(",");
        Serial.print(pose.x_mm, 1);
        Serial.print(",");
        Serial.print(pose.y_mm, 1);
        Serial.print(",");
        Serial.print(stateMachine.getHeading(), 1);
        Serial.print(",");
        Serial.print(readings.front_mm, 1);
        Serial.print(",");
        Serial.print(readings.rear_mm, 1);
        Serial.print(",");
        Serial.print(readings.left_mm, 1);
        Serial.print(",");
        Serial.print(readings.right_mm, 1);
        Serial.print(",");
        Serial.println(readings.us_right_mm, 1);
    }
}

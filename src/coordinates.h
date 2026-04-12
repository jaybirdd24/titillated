#pragma once

#include "fsm.h"
#include "movement.h"
#include "percepetion.h"

class coordinates {
public:
    coordinates(fsm* stateMachine, movement* motors, percepetion* sensors);

    void  update();              // call every loop iteration
    float getX()   const { return x; }   // mm
    float getY()   const { return y; }   // mm
    float getYaw() const;                // degrees (from movement heading)

private:
    fsm*         _fsm;
    movement*    _motors;
    percepetion* _sensors;

    float x, y;
    unsigned long lastUpdateUs;
    RobotState    lastState;
    bool          originSet;

    // Calibration: mm/s per speed unit (PLACEHOLDER — calibrate on real robot)
    static constexpr float FWD_MMPS_PER_UNIT    = 0.5f;
    static constexpr float STRAFE_MMPS_PER_UNIT = 0.45f;

    // Arena dimensions (mm)
    static constexpr float ARENA_W = 1991.0f;
    static constexpr float ARENA_H = 1217.0f;

    // Sensor-to-robot-center offsets (mm) — measure on physical robot
    static constexpr float OFFSET_FRONT = 85.0f;
    static constexpr float OFFSET_REAR  = 85.0f;

    // Commanded speed used by FSM during run phase
    static const int MOVE_SPEED = 200;

    void deadReckon(float dt);
    void checkSensorCorrection(RobotState current);
    void getCommandedVelocity(RobotState current, float& vx, float& vy);
};

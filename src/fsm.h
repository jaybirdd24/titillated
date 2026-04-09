#pragma once

#include "percepetion.h"
#include "movement.h"

enum HomingState {
    HOMING_IDLE = 0,
    HOMING_ROTATE_SCAN,
    HOMING_ROTATE_TO_MIN,
    HOMING_APPROACH_WALL,
    HOMING_SCAN_SIDE_RIGHT,
    HOMING_SCAN_SIDE_LEFT,
    HOMING_ROTATE_TO_SIDE,
    HOMING_APPROACH_SIDE,
    HOMING_LINEUP,
    HOMING_DONE
};

class fsm
{
    private:
        percepetion *perception;
        movement    *motors;

        HomingState   homingState;
        float         fsmHeading;
        unsigned long lastUpdateUs;

        float minUsDist;
        float minUsHeading;
        float scanStartHeading;
        float rightWallCm;
        float rightWallHeading;
        float leftWallCm;
        float leftWallHeading;
        float sideTargetHeading;

        void updateHeading();

    public:
        fsm(percepetion *perception, movement *motors);
        ~fsm();
        void fsmUpdate();
        void fsmHoming();
        HomingState getState()   const { return homingState; }
        float       getHeading() const { return fsmHeading; }
        void startLineup() { lastUpdateUs = micros(); homingState = HOMING_LINEUP; }
};

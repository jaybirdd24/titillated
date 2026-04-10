#pragma once

#include "percepetion.h"
#include "movement.h"

enum HomingState {
    HOMING_IDLE = 0,
    HOMING_ROTATE_SCAN,       // spin 360°, record closest US distance + heading
    HOMING_ROTATE_TO_MIN,     // rotate back to face that wall
    HOMING_APPROACH_WALL,     // move forward until front IR <= 150 mm
    HOMING_APPROACH_SIDE,     // move left until left IR <= 150 mm
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

        void updateHeading();

    public:
        fsm(percepetion *perception, movement *motors);
        ~fsm();
        void fsmUpdate();
        void fsmHoming();
        HomingState getState()   const { return homingState; }
        float       getHeading() const { return fsmHeading; }
};

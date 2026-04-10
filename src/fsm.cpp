#include "fsm.h"
#include <Arduino.h>
#include <math.h>

static const int   ROTATE_SPEED      = 150;
static const int   MOVE_SPEED        = 200;
static const float HEADING_TOL       = 5.0f;
static const float WALL_STOP_MM      = 150.0f;  // 15 cm
static const float SIDE_STOP_MM      = 150.0f;  // 15 cm

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      homingState(HOMING_IDLE),
      fsmHeading(0.0f), lastUpdateUs(0),
      minUsDist(9999.0f), minUsHeading(0.0f)
{
}

fsm::~fsm()
{
}

void fsm::updateHeading()
{
    unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1e6f;
    lastUpdateUs = now;
    fsmHeading += perception->getGyroZ() * (180.0f / PI) * dt;
}

void fsm::fsmUpdate()
{
    fsmHoming();
}

void fsm::fsmHoming()
{
    switch (homingState)
    {
    // ── Step 0: initialise ────────────────────────────────────────────────
    case HOMING_IDLE:
        fsmHeading   = 0.0f;
        minUsDist    = 9999.0f;
        minUsHeading = 0.0f;
        lastUpdateUs = micros();
        homingState  = HOMING_ROTATE_SCAN;
        break;

    // ── Step 1: spin 360°, record heading of closest wall ─────────────────
    case HOMING_ROTATE_SCAN:
    {
        updateHeading();

        float usDist = perception->getUltrasonicCm();
        if (usDist > 0.0f && usDist < minUsDist) {
            minUsDist    = usDist;
            minUsHeading = fsmHeading;
        }

        motors->RotateCCW(ROTATE_SPEED);

        if (fsmHeading >= 360.0f) {
            motors->Stop(true);
            lastUpdateUs = micros();
            homingState  = HOMING_ROTATE_TO_MIN;
        }
        break;
    }

    // ── Step 2: rotate back to face the closest wall ──────────────────────
    case HOMING_ROTATE_TO_MIN:
    {
        updateHeading();

        float err = fsmHeading - minUsHeading;
        if (fabsf(err) <= HEADING_TOL) {
            motors->Stop(true);
            motors->resetHeading();
            lastUpdateUs = micros();
            homingState  = HOMING_APPROACH_WALL;
        } else if (err > 0.0f) {
            motors->RotateCW(ROTATE_SPEED);
        } else {
            motors->RotateCCW(ROTATE_SPEED);
        }
        break;
    }

    // ── Step 3: drive forward until front IR is 15 cm from wall ──────────
    case HOMING_APPROACH_WALL:
    {
        float frontMm = perception->getIRMedFront();
        if (frontMm < WALL_STOP_MM) {
            motors->Stop();
            lastUpdateUs = micros();
            homingState  = HOMING_APPROACH_SIDE;
        } else {
            motors->MoveForward(MOVE_SPEED);
        }
        break;
    }

    // ── Step 4: strafe left until left IR is 15 cm from side wall ────────
    case HOMING_APPROACH_SIDE:
    {
        float leftMm = perception->getIRLongLeft();
        if (leftMm < SIDE_STOP_MM) {
            motors->Stop();
            homingState = HOMING_DONE;
        } else {
            motors->MoveLeft(MOVE_SPEED);
        }
        break;
    }

    case HOMING_DONE:
        break;

    default:
        break;
    }
}

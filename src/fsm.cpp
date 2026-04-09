#include "fsm.h"
#include <Arduino.h>
#include <math.h>

static const int   ROTATE_SPEED  = 150;
static const int   LINEUP_SPEED  = 100;
static const int   MOVE_SPEED    = 200;
static const float WALL_STOP_CM  = 15.0f;
static const float HEADING_TOL   = 5.0f;
static const float LINEUP_TOL    = 5.0f;

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      homingState(HOMING_IDLE),
      fsmHeading(0.0f), lastUpdateUs(0),
      minUsDist(9999.0f), minUsHeading(0.0f),
      scanStartHeading(0.0f),
      rightWallCm(9999.0f), rightWallHeading(0.0f),
      leftWallCm(9999.0f),  leftWallHeading(0.0f),
      sideTargetHeading(0.0f)
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
    case HOMING_IDLE:
        fsmHeading        = 0.0f;
        minUsDist         = 9999.0f;
        minUsHeading      = 0.0f;
        scanStartHeading  = 0.0f;
        rightWallCm       = 9999.0f;
        leftWallCm        = 9999.0f;
        sideTargetHeading = 0.0f;
        lastUpdateUs      = micros();
        homingState       = HOMING_ROTATE_SCAN;
        break;

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

    case HOMING_ROTATE_TO_MIN:
    {
        updateHeading();
        if (fsmHeading <= minUsHeading + HEADING_TOL &&
            fsmHeading >= minUsHeading - HEADING_TOL)
        {
            motors->Stop(true);
            motors->resetHeading();
            homingState = HOMING_APPROACH_WALL;
        }
        else if (fsmHeading > minUsHeading)
        {
            motors->RotateCW(ROTATE_SPEED);
        }
        else
        {
            motors->RotateCCW(ROTATE_SPEED);
        }
        break;
    }

    case HOMING_APPROACH_WALL:
    {
        motors->MoveRight(MOVE_SPEED);
        float usDist = perception->getUltrasonicCm();
        if (usDist > 0.0f && usDist <= WALL_STOP_CM) {
            motors->Stop();
            scanStartHeading = fsmHeading;
            lastUpdateUs     = micros();
            homingState      = HOMING_SCAN_SIDE_RIGHT;
        }
        break;
    }

    case HOMING_SCAN_SIDE_RIGHT:
    {
        updateHeading();
        float targetRight = scanStartHeading - 90.0f;
        if (fsmHeading <= targetRight + HEADING_TOL &&
            fsmHeading >= targetRight - HEADING_TOL)
        {
            motors->Stop();
            rightWallCm      = perception->getUltrasonicCm();
            rightWallHeading = fsmHeading;
            lastUpdateUs     = micros();
            homingState      = HOMING_SCAN_SIDE_LEFT;
        }
        else if (fsmHeading > targetRight)
        {
            motors->RotateCW(ROTATE_SPEED);
        }
        else
        {
            motors->RotateCCW(ROTATE_SPEED);
        }
        break;
    }

    case HOMING_SCAN_SIDE_LEFT:
    {
        updateHeading();
        float targetLeft = scanStartHeading + 90.0f;
        if (fsmHeading >= targetLeft - HEADING_TOL &&
            fsmHeading <= targetLeft + HEADING_TOL)
        {
            motors->Stop();
            leftWallCm      = perception->getUltrasonicCm();
            leftWallHeading = fsmHeading;
            sideTargetHeading = (leftWallCm <= rightWallCm)
                                ? leftWallHeading
                                : rightWallHeading;
            lastUpdateUs = micros();
            homingState  = HOMING_ROTATE_TO_SIDE;
        }
        else if (fsmHeading < targetLeft)
        {
            motors->RotateCCW(ROTATE_SPEED);
        }
        else
        {
            motors->RotateCW(ROTATE_SPEED);
        }
        break;
    }

    case HOMING_ROTATE_TO_SIDE:
    {
        updateHeading();
        if (fsmHeading <= sideTargetHeading + HEADING_TOL &&
            fsmHeading >= sideTargetHeading - HEADING_TOL)
        {
            motors->Stop();
            motors->resetHeading();
            homingState = HOMING_APPROACH_SIDE;
        }
        else if (fsmHeading > sideTargetHeading)
        {
            motors->RotateCW(ROTATE_SPEED);
        }
        else
        {
            motors->RotateCCW(ROTATE_SPEED);
        }
        break;
    }

    case HOMING_APPROACH_SIDE:
    {
        motors->MoveRight(MOVE_SPEED);
        float usDist = perception->getUltrasonicCm();
        if (usDist > 0.0f && usDist <= WALL_STOP_CM) {
            motors->Stop();
            lastUpdateUs = micros();
            homingState  = HOMING_LINEUP;
        }
        break;
    }

    case HOMING_LINEUP:
    {
        updateHeading();
        float frontIR_mm = perception->getIRMedFront();
        float usDist     = perception->getUltrasonicCm();
        if (usDist <= 0.0f) {
            motors->Stop();
            break;
        }
        float us_mm = usDist * 10.0f;
        float diff  = frontIR_mm - us_mm;
        if (fabsf(diff) <= LINEUP_TOL) {
            motors->Stop();
            homingState = HOMING_DONE;
        } else if (diff < 0.0f) {
            motors->RotateCCW(LINEUP_SPEED);
        } else {
            motors->RotateCW(LINEUP_SPEED);
        }
        break;
    }

    case HOMING_DONE:
        break;

    default:
        break;
    }
}

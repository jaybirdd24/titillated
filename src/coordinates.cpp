#include "coordinates.h"
#include <Arduino.h>
#include <math.h>

coordinates::coordinates(fsm* stateMachine, movement* motors, percepetion* sensors)
    : _fsm(stateMachine), _motors(motors), _sensors(sensors),
      x(0.0f), y(0.0f), lastUpdateUs(0),
      lastState(HOMING_IDLE), originSet(false)
{
}

float coordinates::getYaw() const {
    return _motors->getHeading();
}

// ── Main update (call every loop) ────────────────────────────────────────────

void coordinates::update() {
    unsigned long now = micros();
    if (lastUpdateUs == 0) {
        lastUpdateUs = now;
        return;
    }
    float dt = (now - lastUpdateUs) / 1e6f;
    lastUpdateUs = now;

    RobotState current = _fsm->getState();

    // Detect homing → run transition: set origin
    if (!originSet) {
        if (lastState == HOMING_APPROACH_FWD && current == RUN_MOVE_DOWN) {
            x = 0.0f;
            y = 0.0f;
            originSet = true;
        }
        lastState = current;
        return;
    }

    // Dead reckon position
    deadReckon(dt);

    // Apply sensor corrections on state transitions
    if (current != lastState) {
        checkSensorCorrection(current);
    }

    lastState = current;
}

// ── Dead reckoning ───────────────────────────────────────────────────────────

void coordinates::getCommandedVelocity(RobotState current, float& vx, float& vy) {
    // vx = strafe axis (+X = leftward), vy = forward axis (+Y = downward)
    switch (current) {
        case RUN_MOVE_DOWN:
        case RUN_FINAL_MOVE_DOWN:
            vx = 0.0f;
            vy = MOVE_SPEED * FWD_MMPS_PER_UNIT;    // moving down = +Y
            break;
        case RUN_MOVE_UP:
        case RUN_FINAL_MOVE_UP:
            vx = 0.0f;
            vy = -MOVE_SPEED * FWD_MMPS_PER_UNIT;   // moving up = -Y
            break;
        case RUN_STRAFE_LEFT_A:
        case RUN_STRAFE_LEFT_B:
            vx = MOVE_SPEED * STRAFE_MMPS_PER_UNIT;  // strafe left = +X
            vy = 0.0f;
            break;
        default:
            vx = 0.0f;
            vy = 0.0f;
            break;
    }
}

void coordinates::deadReckon(float dt) {
    float vx_mms, vy_mms;
    getCommandedVelocity(_fsm->getState(), vx_mms, vy_mms);

    if (vx_mms == 0.0f && vy_mms == 0.0f) return;

    // Rotate by heading to get world-frame displacement
    float h = _motors->getHeading() * (PI / 180.0f);
    float cos_h = cosf(h);
    float sin_h = sinf(h);

    x += (vx_mms * cos_h - vy_mms * sin_h) * dt;
    y += (vx_mms * sin_h + vy_mms * cos_h) * dt;
}

// ── Sensor corrections at wall contacts ──────────────────────────────────────

void coordinates::checkSensorCorrection(RobotState current) {
    // Rear wall contact: snap Y to arena height minus sensor offset
    if ((lastState == RUN_MOVE_DOWN && current == RUN_STRAFE_LEFT_A) ||
        (lastState == RUN_FINAL_MOVE_DOWN && current == STATE_DONE)) {
        float rearMm = _sensors->getIRLongRear();
        if (rearMm > 0.0f) {
            y = ARENA_H - rearMm - OFFSET_REAR;
        }
    }

    // Front wall contact: snap Y to sensor reading plus offset (near Y=0)
    if ((lastState == RUN_MOVE_UP && current == RUN_STRAFE_LEFT_B) ||
        (lastState == RUN_FINAL_MOVE_UP && current == STATE_DONE)) {
        float frontMm = _sensors->getIRMedFront();
        if (frontMm > 0.0f) {
            y = frontMm + OFFSET_FRONT;
        }
    }
}

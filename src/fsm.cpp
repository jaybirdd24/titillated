#include "fsm.h"

#include <Arduino.h>
#include <math.h>

#include "robot_config.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   ROTATE_SPEED       = 150;
static const float US_SPIKE_THRESHOLD = 60000.0f;
static const int   US_WARMUP_READINGS = 5;
static const float RETURN_MIN_DEG     = 20.0f;
static const int   RETURN_EXTRA_MS    = 100;
static const float APPROACH_STOP_CM   = 15.0f;
static const int   APPROACH_SPEED     = 200;
static const float FORWARD_STOP_MM    = 150.0f;
static const int   FORWARD_SPEED      = 200;

static const int   MOVE_SPEED         = 200;
static const float REAR_STOP_MM       = 150.0f;
static const float FRONT_STOP_MM      = 150.0f;
static const float LEFT_IGNORE_MM     = 110.0f;
static const int   LEFT_CONFIRM_N     = 20;
static const int   LEFT_RESET_N       = 10;
// ─────────────────────────────────────────────────────────────────────────────

namespace {

float wrapDegrees(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

bool isValid(float value, float min_value, float max_value)
{
    return value >= min_value && value <= max_value;
}

float bestRightWallCenterEstimateMm(percepetion *perception)
{
    const float right_ir_mm = perception->getIRMedRight();
    const float us_mm = perception->getUltrasonicMm();

    const bool right_ok = isValid(right_ir_mm, robot_config::kMedIRMinMm, robot_config::kMedIRMaxMm);
    const bool us_ok = isValid(us_mm, robot_config::kUsMinMm, robot_config::kUsMaxMm);

    float estimates[2];
    int count = 0;

    if (right_ok && us_ok) {
        estimates[count++] = right_ir_mm - robot_config::kRightIRMount.left_offset_mm;
        estimates[count++] = us_mm - robot_config::kRightUSMount.left_offset_mm;
    } else if (right_ok) {
        estimates[count++] = right_ir_mm - robot_config::kRightIRMount.left_offset_mm;
    } else if (us_ok) {
        estimates[count++] = us_mm - robot_config::kRightUSMount.left_offset_mm;
    }

    if (count == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (int i = 0; i < count; ++i) {
        sum += estimates[i];
    }
    return sum / (float)count;
}

}  // namespace

fsm::fsm(percepetion *perception, movement *motors, pose_estimator *estimator)
    : perception(perception), motors(motors), estimator(estimator),
      state(HOMING_IDLE),
      scanAccumulatedDeg(0.0f), returnAccumulatedDeg(0.0f),
      previousHeadingDeg(0.0f), previousHeadingValid(false),
      topCount(0), minUsDist(9999.0f), minUsHeading(0.0f),
      lastValidUs(-1.0f), usReadingCount(0), lastSampleMs(0),
      returnExtraStart(-1),
      strafeTargetX(0.0f),
      leftWallSeen(false), leftNonIgnoreCount(0),
      leftIgnoreCount(0), leftLastCountedMs(0)
{
}

fsm::~fsm() {}

float fsm::getHeading() const
{
    return estimator != nullptr ? estimator->getHeadingDeg() : 0.0f;
}

Pose2D fsm::getPose() const
{
    return estimator != nullptr ? estimator->getPose() : Pose2D{0.0f, 0.0f, 0.0f};
}

float fsm::headingDeltaDeg()
{
    const float current_heading = getHeading();
    if (!previousHeadingValid) {
        previousHeadingDeg = current_heading;
        previousHeadingValid = true;
        return 0.0f;
    }

    const float delta = wrapDegrees(current_heading - previousHeadingDeg);
    previousHeadingDeg = current_heading;
    return delta;
}

// ── Top-N helpers ─────────────────────────────────────────────────────────────

void fsm::insertTopN(float dist, float head) {
    if (topCount < TOP_N) {
        topDist[topCount] = dist;
        topHead[topCount] = head;
        topCount++;
    } else {
        int worst = 0;
        for (int i = 1; i < TOP_N; i++)
            if (topDist[i] > topDist[worst]) worst = i;
        if (dist < topDist[worst]) {
            topDist[worst] = dist;
            topHead[worst] = head;
        }
    }
}

float fsm::avgTopHeading() {
    if (topCount == 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < topCount; i++) sum += topHead[i];
    return sum / topCount;
}

float fsm::avgTopDist() {
    if (topCount == 0) return 9999.0f;
    float sum = 0.0f;
    for (int i = 0; i < topCount; i++) sum += topDist[i];
    return sum / topCount;
}

void fsm::initializePoseFromHoming()
{
    if (estimator == nullptr) {
        return;
    }

    float front_mm = perception->getIRMedFront();
    if (!isValid(front_mm, robot_config::kMedIRMinMm, robot_config::kMedIRMaxMm)) {
        front_mm = FORWARD_STOP_MM;
    }

    float x_mm = bestRightWallCenterEstimateMm(perception);
    if (!isValid(x_mm, robot_config::kUsMinMm, robot_config::kArenaWidthMm)) {
        x_mm = APPROACH_STOP_CM * 10.0f - robot_config::kRightUSMount.left_offset_mm;
    }

    const float y_mm = front_mm + robot_config::kFrontIRMount.forward_offset_mm;

    estimator->resetPose({x_mm, y_mm, 0.0f}, 15.0f, 2.0f * robot_config::kDegToRad);
    motors->resetHeading();
    motors->setTargetHeading(0.0f);
}

// ── Left wall detection ───────────────────────────────────────────────────────

bool fsm::leftWallDetected() {
    float leftMm = perception->getIRLongLeft();
    unsigned long now = millis();
    if (now - leftLastCountedMs < 50) return false;
    leftLastCountedMs = now;

    if (!leftWallSeen) {
        if (leftMm <= LEFT_IGNORE_MM) {
            leftNonIgnoreCount = 0;
        } else {
            leftNonIgnoreCount++;
            if (leftNonIgnoreCount >= LEFT_CONFIRM_N) {
                leftWallSeen    = true;
                leftIgnoreCount = 0;
                Serial.println("Left wall seen -- waiting for 10x ignore");
            }
        }
        return false;
    } else {
        if (leftMm <= LEFT_IGNORE_MM) {
            leftIgnoreCount++;
        } else {
            leftIgnoreCount = 0;
        }
        return leftIgnoreCount >= LEFT_RESET_N;
    }
}

// ── Main update ───────────────────────────────────────────────────────────────

void fsm::fsmUpdate() {
    if (state <= HOMING_APPROACH_FWD)
        doHoming();
    else
        doRun();
}

// ── Homing ────────────────────────────────────────────────────────────────────

void fsm::doHoming() {
    switch (state) {

    case HOMING_IDLE:
        if (estimator != nullptr) {
            estimator->begin();
        }
        motors->resetHeading();
        scanAccumulatedDeg = 0.0f;
        returnAccumulatedDeg = 0.0f;
        previousHeadingValid = false;
        minUsDist      = 9999.0f;
        minUsHeading   = 0.0f;
        lastValidUs    = -1.0f;
        usReadingCount = 0;
        topCount       = 0;
        lastSampleMs   = 0;
        returnExtraStart = -1;
        leftWallSeen = false;
        leftNonIgnoreCount = 0;
        leftIgnoreCount = 0;
        state = HOMING_SCAN;
        break;

    case HOMING_SCAN:
        {
            const float delta = headingDeltaDeg();
            if (delta > 0.0f) {
                scanAccumulatedDeg += delta;
            }

            unsigned long now = millis();
            if (now - lastSampleMs >= 100) {
                lastSampleMs = now;
                float usDist = perception->getUltrasonicCm();
                usReadingCount++;
                bool valid = (usReadingCount > US_WARMUP_READINGS) &&
                             (usDist > 0.0f) &&
                             (lastValidUs < 0.0f || fabsf(usDist - lastValidUs) <= US_SPIKE_THRESHOLD);
                if (valid) {
                    lastValidUs = usDist;
                    insertTopN(usDist, getHeading());
                    if (usDist < minUsDist) minUsDist = usDist;
                }
            }
        }
        motors->RotateCCW(ROTATE_SPEED);
        if (scanAccumulatedDeg >= 360.0f) {
            motors->Stop(true);
            previousHeadingValid = false;
            returnAccumulatedDeg = 0.0f;
            returnExtraStart = -1;
            minUsHeading = avgTopHeading();
            Serial.print("Scan done. Avg dist=");
            Serial.print(avgTopDist(), 1);
            Serial.print(" heading=");
            Serial.println(minUsHeading, 1);
            state = HOMING_RETURN;
        }
        break;

    case HOMING_RETURN:
        {
            const float delta = headingDeltaDeg();
            if (delta < 0.0f) {
                returnAccumulatedDeg += -delta;
            }

            float usDist  = perception->getUltrasonicCm();
            if (returnAccumulatedDeg >= RETURN_MIN_DEG &&
                usDist > 0.0f &&
                usDist <= minUsDist + 2.0f) {
                if (returnExtraStart < 0) returnExtraStart = millis();
                if (millis() - (unsigned long)returnExtraStart >= (unsigned long)RETURN_EXTRA_MS) {
                    motors->Stop(true);
                    Serial.println("Facing wall -- moving right");
                    state = HOMING_APPROACH_WALL;
                } else {
                    motors->RotateCW(ROTATE_SPEED);
                }
            } else {
                motors->RotateCW(ROTATE_SPEED);
            }
        }
        break;

    case HOMING_APPROACH_WALL:
        {
            float usDist = perception->getUltrasonicCm();
            if (usDist > 0.0f && usDist < APPROACH_STOP_CM) {
                motors->Stop(true);
                Serial.println("At side wall -- moving forward");
                state = HOMING_APPROACH_FWD;
            } else {
                motors->MoveRight(APPROACH_SPEED);
            }
        }
        break;

    case HOMING_APPROACH_FWD:
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm < FORWARD_STOP_MM) {
                motors->Stop(true);
                initializePoseFromHoming();
                Serial.println("At front wall -- starting run");
                state = RUN_MOVE_DOWN;
            } else {
                motors->MoveForward(FORWARD_SPEED);
            }
        }
        break;

    default:
        break;
    }
}

// ── Run ───────────────────────────────────────────────────────────────────────

void fsm::doRun() {
    switch (state) {

    case RUN_MOVE_DOWN:
        if (leftWallDetected()) {
            motors->Stop(true);
            Serial.println("Left wall -- final move down");
            state = RUN_FINAL_MOVE_DOWN;
            break;
        }
        {
            float rearMm = perception->getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors->Stop(true);
                strafeTargetX = getPose().x_mm + robot_config::kLaneStepMm;
                state = RUN_STRAFE_LEFT_A;
            } else {
                motors->MoveBackward(MOVE_SPEED);
            }
        }
        break;

    case RUN_STRAFE_LEFT_A:
        if (getPose().x_mm >= strafeTargetX) {
            motors->Stop(true);
            state = RUN_MOVE_UP;
        } else {
            motors->MoveLeft(MOVE_SPEED);
        }
        break;

    case RUN_MOVE_UP:
        if (leftWallDetected()) {
            motors->Stop(true);
            Serial.println("Left wall -- final move up");
            state = RUN_FINAL_MOVE_UP;
            break;
        }
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors->Stop(true);
                strafeTargetX = getPose().x_mm + robot_config::kLaneStepMm;
                state = RUN_STRAFE_LEFT_B;
            } else {
                motors->MoveForward(MOVE_SPEED);
            }
        }
        break;

    case RUN_STRAFE_LEFT_B:
        if (getPose().x_mm >= strafeTargetX) {
            motors->Stop(true);
            state = RUN_MOVE_DOWN;
        } else {
            motors->MoveLeft(MOVE_SPEED);
        }
        break;

    case RUN_FINAL_MOVE_DOWN:
        {
            float rearMm = perception->getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors->Stop(true);
                Serial.println("DONE");
                state = STATE_DONE;
            } else {
                motors->MoveBackward(MOVE_SPEED);
            }
        }
        break;

    case RUN_FINAL_MOVE_UP:
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors->Stop(true);
                Serial.println("DONE");
                state = STATE_DONE;
            } else {
                motors->MoveForward(MOVE_SPEED);
            }
        }
        break;

    case STATE_DONE:
        motors->Stop(true);
        break;

    default:
        break;
    }
}

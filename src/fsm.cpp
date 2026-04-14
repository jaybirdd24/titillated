#include "fsm.h"
#include <Arduino.h>
#include <math.h>

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
static const float LEFT_STOP_MM       = 120.0f;
static const float LEFT_IGNORE_MM     = 110.0f;
static const int   LEFT_CONFIRM_N     = 20;
static const int   LEFT_RESET_N       = 10;
static const unsigned long STRAFE_TIME_MS = 800;

static const float SQUARE_DIFF         =  0.0f;  // us_mm - ir_mm when square; calibrate this
static const float SQUARE_KP           =  1.5f;  // proportional gain
static const float SQUARE_MAX_SPEED    = 80.0f;  // max wz rotation speed
static const float SQUARE_THRESHOLD_MM =  5.0f;  // error tolerance (mm)
static const int   SQUARE_CONFIRM_N    = 15;      // consecutive reads within threshold to confirm
// ─────────────────────────────────────────────────────────────────────────────

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      state(HOMING_IDLE),
      heading(0.0f), lastUpdateUs(0),
      topCount(0), minUsDist(9999.0f), minUsHeading(0.0f),
      lastValidUs(-1.0f), usReadingCount(0), lastSampleMs(0),
      returnStartHeading(0.0f), returnExtraStart(-1), lastSampleUsMs(0),
      squareConfirmCount(0), squareLastPrintMs(0),
      strafeStart(0),
      leftWallSeen(false), leftNonIgnoreCount(0),
      leftIgnoreCount(0), leftLastCountedMs(0)
{
}

fsm::~fsm() {}

// ── Heading ───────────────────────────────────────────────────────────────────

void fsm::updateHeading() {
    unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1e6f;
    lastUpdateUs = now;
    heading += perception->getGyroZ() * (180.0f / PI) * dt;
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
                Serial.println("Left wall seen — waiting for 10x ignore");
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
    if (state <= HOMING_SQUARE_UP)
        doHoming();
    else
        doRun();
}

// ── Homing ────────────────────────────────────────────────────────────────────

void fsm::doHoming() {
    switch (state) {

    case HOMING_IDLE:
        heading        = 0.0f;
        minUsDist      = 9999.0f;
        minUsHeading   = 0.0f;
        lastValidUs    = -1.0f;
        usReadingCount = 0;
        topCount       = 0;
        lastUpdateUs   = micros();
        state          = HOMING_SCAN;
        break;

    case HOMING_SCAN:
        updateHeading();
        {
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
                    insertTopN(usDist, heading);
                    if (usDist < minUsDist) minUsDist = usDist;
                }
            }
        }
        motors->RotateCCW(ROTATE_SPEED);
        if (heading >= 360.0f) {
            motors->Stop(true);
            lastUpdateUs      = micros();
            returnStartHeading = heading;
            returnExtraStart  = -1;
            minUsHeading      = avgTopHeading();
            Serial.print("Scan done. Avg dist=");
            Serial.print(avgTopDist(), 1);
            Serial.print(" heading=");
            Serial.println(minUsHeading, 1);
            state = HOMING_RETURN;
        }
        break;

    case HOMING_RETURN:
        updateHeading();
        {
            float rotated = returnStartHeading - heading;
            float usDist  = perception->getUltrasonicCm();
            if (rotated >= RETURN_MIN_DEG && usDist > 0.0f && usDist <= minUsDist + 2.0f) {
                if (returnExtraStart < 0) returnExtraStart = millis();
                if (millis() - (unsigned long)returnExtraStart >= (unsigned long)RETURN_EXTRA_MS) {
                    motors->Stop(true);
                    Serial.println("Facing wall — moving right");
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
                Serial.println("At side wall — moving forward");
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
                squareConfirmCount = 0;
                Serial.println("At front wall — squaring up");
                state = HOMING_SQUARE_UP;
            } else {
                motors->MoveForward(FORWARD_SPEED);
            }
        }
        break;

    case HOMING_SQUARE_UP:
    {
        float us_mm = perception->getUltrasonicCm() * 10.0f;
        float ir_mm = perception->getIRMedRight();
        float error = (us_mm - ir_mm) - SQUARE_DIFF;

        unsigned long now = millis();
        if (now - squareLastPrintMs >= 200) {
            squareLastPrintMs = now;
            Serial.print("SQ us="); Serial.print(us_mm, 1);
            Serial.print(" ir="); Serial.print(ir_mm, 1);
            Serial.print(" err="); Serial.println(error, 1);
        }

        if (fabsf(error) < SQUARE_THRESHOLD_MM) {
            squareConfirmCount++;
            if (squareConfirmCount >= SQUARE_CONFIRM_N) {
                motors->Stop(true);
                Serial.println("Squared up — starting run");
                state = RUN_MOVE_DOWN;
            }
        } else {
            squareConfirmCount = 0;
        }

        float wz = constrain(SQUARE_KP * error, -SQUARE_MAX_SPEED, SQUARE_MAX_SPEED);
        motors->drive(0, 0, (int)wz);
        break;
    }

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
            Serial.println("Left wall — final move down");
            state = RUN_FINAL_MOVE_DOWN;
            break;
        }
        {
            float rearMm = perception->getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors->Stop(true);
                strafeStart = millis();
                state = RUN_STRAFE_LEFT_A;
            } else {
                motors->MoveBackward(MOVE_SPEED);
            }
        }
        break;

    case RUN_STRAFE_LEFT_A:
        if (millis() - strafeStart >= STRAFE_TIME_MS) {
            motors->Stop(true);
            state = RUN_MOVE_UP;
        } else {
            motors->MoveLeft(MOVE_SPEED);
        }
        break;

    case RUN_MOVE_UP:
        if (leftWallDetected()) {
            motors->Stop(true);
            Serial.println("Left wall — final move up");
            state = RUN_FINAL_MOVE_UP;
            break;
        }
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors->Stop(true);
                strafeStart = millis();
                state = RUN_STRAFE_LEFT_B;
            } else {
                motors->MoveForward(MOVE_SPEED);
            }
        }
        break;

    case RUN_STRAFE_LEFT_B:
        if (millis() - strafeStart >= STRAFE_TIME_MS) {
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

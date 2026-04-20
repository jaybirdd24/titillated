#include "fsm.h"
#include <Arduino.h>
#include <math.h>

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   ROTATE_SPEED       = 150;
static const float US_SPIKE_THRESHOLD = 60000.0f;
static const int   US_WARMUP_READINGS = 5;
static const float RETURN_MIN_DEG     = 20.0f;
static const int   RETURN_EXTRA_MS    = 100;


static const float APPROACH_STOP_CM   = 15.0f;//change this for better score


static const int   APPROACH_SPEED     = 200;
static const float FORWARD_STOP_MM    = 100.0f;//and this one
static const int   FORWARD_SPEED      = 250;



static const float LEFT_STOP_MM       = 150.0f;


static const int   MOVE_SPEED         = 300;
static const float REAR_STOP_MM       = 100.0f;//and this one
static const float FRONT_STOP_MM      = 100.0f;///and this one
static const float LEFT_IGNORE_MM     = 110.0f;
static const int   LEFT_CONFIRM_N     = 20;
static const int   LEFT_RESET_N       = 10;
static const unsigned long STRAFE_TIME_MS = 693;//tune the strafe time between cuts
static const float STRAFE_DECEL_KP       =   3.0f; // ramp down strafe speed near target
static const float STRAFE_MIN_SPEED      =  60.0f;  // don't go slower than this during strafe

static const int   RAM_SPEED              = 200;     // speed to drive into wall
static const unsigned long RAM_TIME_MS    = 1000;    // how long to press against wall
static const float BACK_OFF_TARGET_MM     = 86.0f;   // IR med right target after backing off
static const float BACK_OFF_TOLERANCE_MM  = 7.0f;    // close enough
static const float BACK_OFF_KP            = 11.5f;
static const float BACK_OFF_MAX_SPEED     = 120.0f;
static const float BACK_OFF_MIN_SPEED     = 40.0f;
static const unsigned long BACK_OFF_HOLD_MS = 250;   // hold in range before starting run

static const float WF_SETPOINT_CM = 9.5f;//and this one



static const float WF_KP          = 30.0f;
static const float WF_KI          =  0.5f;
static const float WF_MAX_INT     = 200.0f;

// ─────────────────────────────────────────────────────────────────────────────

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      state(HOMING_IDLE),
      heading(0.0f), lastUpdateUs(0),
      topCount(0), minUsDist(9999.0f), minUsHeading(0.0f),
      lastValidUs(-1.0f), usReadingCount(0), lastSampleMs(0),
      returnStartHeading(0.0f), returnExtraStart(-1), lastSampleUsMs(0),
      squareInRangeStart(-1), squareLastPrintMs(0), squareUsSmoothed(-1.0f),
      squareIntegral(0.0f), squarePrevError(0.0f), squareLastUs(0),
      ramStartMs(0), backOffInRangeStart(-1),
      wf_integral(0.0f), wf_last_us(0),
      strafeStartMs(0), runHeading(0.0f), runWfSetpoint(0.0f),
      leftWallSeen(false), leftNonIgnoreCount(0),
      leftIgnoreCount(0), leftLastCountedMs(0),
      firstRun(true),
      runCount(1)
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

// ── Wall-follow correction (APPROACH_FWD) ────────────────────────────────────

int fsm::wfCorrection() {
    unsigned long now = micros();
    float dt = (now - wf_last_us) / 1e6f;
    wf_last_us = now;
    if (dt > 0.1f) dt = 0.1f;

    float dist = perception->getUltrasonicCm();
    if (dist <= 0.0f) return 0;

    float error = dist - WF_SETPOINT_CM;
    wf_integral += error * dt;
    wf_integral = constrain(wf_integral, -WF_MAX_INT, WF_MAX_INT);

    float vy = WF_KP * error + WF_KI * wf_integral;
    return (int)constrain(-vy, -300, 300);
}

// ── Main update ───────────────────────────────────────────────────────────────

void fsm::fsmUpdate() {
    if (state <= HOMING_BACK_OFF)
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
                ramStartMs = millis();
                Serial.println("At front wall — ramming right wall to square up");
                state = HOMING_RAM_WALL;
            } else {
                int vy = wfCorrection();
                motors->drive(FORWARD_SPEED, vy, (int)motors->headingCorrection());
            }
        }
        break;

    case HOMING_RAM_WALL:
        if (millis() - ramStartMs >= RAM_TIME_MS) {
            motors->Stop(true);
            backOffInRangeStart = -1;
            Serial.println("Ram done — backing off to 86 mm");
            state = HOMING_BACK_OFF;
        } else {
            motors->MoveRight(RAM_SPEED);
        }
        break;

    case HOMING_BACK_OFF:
        {
            float ir_mm = perception->getIRMedRight();
            if (ir_mm <= 0.0f) break;

            float error = -ir_mm + BACK_OFF_TARGET_MM;
            unsigned long now = millis();

            if (fabsf(error) < BACK_OFF_TOLERANCE_MM) {
                motors->Stop(true);
                if (backOffInRangeStart < 0) backOffInRangeStart = (long)now;
                if (now - (unsigned long)backOffInRangeStart >= BACK_OFF_HOLD_MS) {
                    runHeading = motors->getHeading();
                    runWfSetpoint = ir_mm;
                    motors->resetWallFollow();
                    Serial.print("Squared up — backed off to ");
                    Serial.print(ir_mm, 1);
                    Serial.println(" mm — starting run");
                    state = RUN_MOVE_DOWN;
                }
            } else {
                backOffInRangeStart = -1;
                int speed = (int)constrain(fabsf(BACK_OFF_KP * error),
                                           BACK_OFF_MIN_SPEED, BACK_OFF_MAX_SPEED);
                if (error > 0.0f)
                    motors->MoveLeft(speed);   // too far from wall → move left (away)
                else
                    motors->MoveRight(speed);  // too close → move right (closer)
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
        {
            float rearMm = perception->getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors->Stop(true);
                motors->setTargetHeading(runHeading);
                strafeStartMs = millis();
                Serial.println("Rear wall — strafing left");
                firstRun = false;
                state = RUN_STRAFE_LEFT_A;
            } else {
                if (firstRun) {
                    int vy = (int)motors->wallFollowCorrection(90.0f);
                    motors->drive(-MOVE_SPEED, vy, (int)motors->headingCorrection());
                } else {
                    motors->drive(-MOVE_SPEED, 0, (int)motors->headingCorrection());
                }
            }
        }
        break;

    case RUN_STRAFE_LEFT_A:
        {

        }
        if (millis() - strafeStartMs >= STRAFE_TIME_MS) {
            motors->Stop(true);
            motors->setTargetHeading(runHeading);
            runCount++;
            Serial.print("Strafe done — move up (run ");
            Serial.print(runCount);
            Serial.println(")");
                        if (runCount >= 10) {
                motors->Stop(true);
                motors->setTargetHeading(runHeading);
                Serial.println("Run count >= 9 — final move up");
                state = RUN_FINAL_MOVE_UP;
                break;
            }
            state = RUN_MOVE_UP;
        } else {
            motors->MoveLeft(MOVE_SPEED);
        }
        break;

    case RUN_MOVE_UP:
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors->Stop(true);
                motors->setTargetHeading(runHeading);
                strafeStartMs = millis();
                Serial.println("Front wall — strafing left");
                state = RUN_STRAFE_LEFT_B;
            } else {
                motors->drive(MOVE_SPEED, 0, (int)motors->headingCorrection());
            }
        }
        break;

    case RUN_STRAFE_LEFT_B:
        {

        }
        if (millis() - strafeStartMs >= STRAFE_TIME_MS) {
            motors->Stop(true);
            motors->setTargetHeading(runHeading);
            runCount++;
            Serial.print("Strafe done — move down (run ");
            Serial.print(runCount);
            Serial.println(")");
                        if (runCount >= 10) {
                motors->Stop(true);
                motors->setTargetHeading(runHeading);
                Serial.println("Run count >= 9 — final move down");
                state = RUN_FINAL_MOVE_DOWN;
                break;
            }
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
                int vy = (int)motors->wallFollowCorrection(93.0f, true);
                motors->drive(-MOVE_SPEED, vy, (int)motors->headingCorrection());
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
                int vy = (int)motors->wallFollowCorrection(93.0f, true);
                motors->drive(MOVE_SPEED, vy, (int)motors->headingCorrection());
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

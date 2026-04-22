#include "fsm.h"
#include <Arduino.h>
#include <math.h>

// ── Tuning ────────────────────────────────────────────────────────────────────
// ── Scan / return tuning ─────────────────────────────────────────────────────
static const int   SCAN_SPIN_SPEED        = 150;
static const int   SCAN_SAMPLE_INTERVAL   = 50;       // ms between samples (20 Hz)
static const float US_MIN_CM              = 2.0f;
static const float US_MAX_CM              = 130.0f;
static const float TROUGH_BAND_CM         = 1.5f;     // distance above min that counts as trough
static const int   MIN_TROUGH_SAMPLES     = 3;
const float OPPOSITE_TOL_DEG = 20.0f;

static const int   RETURN_SPIN_FAST       = 110;
static const int   RETURN_SPIN_SLOW       = 90;
static const float RETURN_SLOW_BAND_DEG   = 10.0f;
static const float RETURN_STOP_TOL_DEG    = 2.0f;
static const unsigned long RETURN_STABLE_MS = 250;


static const float APPROACH_STOP_CM   = 15.0f;//change this for better score


static const int   APPROACH_SPEED     = 200;
static const float FORWARD_STOP_MM    = 100.0f;//and this one
static const int   FORWARD_SPEED      = 250;



static const float LEFT_STOP_MM       = 150.0f;


static const int   MOVE_SPEED         = 400;
static const float REAR_STOP_MM       = 100.0f;//and this one
static const float FRONT_STOP_MM      = 100.0f;///and this one
static const float LEFT_IGNORE_MM     = 110.0f;
static const int   LEFT_CONFIRM_N     = 20;
static const int   LEFT_RESET_N       = 10;
static const unsigned long STRAFE_TIME_MS = 525;//tune the strafe time between cuts
static const float STRAFE_DECEL_KP       =   3.0f; // ramp down strafe speed near target
static const float STRAFE_MIN_SPEED      =  60.0f;  // don't go slower than this during strafe

static const int   RAM_SPEED              = 200;     // speed to drive into wall
static const unsigned long RAM_TIME_MS    = 1500;    // how long to press against wall
static const float BACK_OFF_TARGET_MM     = 86.0f;   // IR med right target after backing off
static const float BACK_OFF_TOLERANCE_MM  = 9.0f;    // close enough
static const float BACK_OFF_KP            = 9.5f;
static const float BACK_OFF_MAX_SPEED     = 120.0f;
static const float BACK_OFF_MIN_SPEED     = 40.0f;
static const unsigned long BACK_OFF_HOLD_MS = 250;   // hold in range before starting run

static const float WF_SETPOINT_CM = 9.5f;//and this one
static const float WF_TOLERANCE_CM  = 1.0f;

static const float WF_KP          = 30.0f;
static const float WF_KI          =  0.5f;
static const float WF_MAX_INT     = 200.0f;



// ────────────────────────────────────────────────────────────────────────────

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      state(HOMING_IDLE),
      heading(0.0f), lastUpdateUs(0),
      scanCount(0), scanStartHeading(0.0f),
      usFilterCount(0), usFilterHead(0),
      scanTargetHeading(0.0f), chosenMinDistCm(-1.0f), globalMinIdx(-1),
      returnInTolStart(0), returnInTolActive(false),
      lastSampleMs(0),
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

// ── US moving-average filter ─────────────────────────────────────────────────

void fsm::resetUsFilter() {
    usFilterCount = 0;
    usFilterHead  = 0;
    for (int i = 0; i < US_FILTER_SIZE; i++) usFilterBuf[i] = -1.0f;
}

float fsm::usMovingAverage(float us) {
    usFilterBuf[usFilterHead] = us;
    usFilterHead = (usFilterHead + 1) % US_FILTER_SIZE;
    if (usFilterCount < US_FILTER_SIZE) usFilterCount++;

    float sum = 0.0f;
    int valid = 0;
    for (int i = 0; i < usFilterCount; i++) {
        if (usFilterBuf[i] >= US_MIN_CM && usFilterBuf[i] <= US_MAX_CM) {
            sum += usFilterBuf[i];
            valid++;
        }
    }
    return (valid > 0) ? sum / (float)valid : -1.0f;
}

// ── Scan analysis ────────────────────────────────────────────────────────────

float wrap360(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

float angleDiffDeg(float a, float b) {
    float d = wrap360(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

float signedAngleErrorDeg(float target, float current) {
    float e = target - current;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

float circularMeanDeg(float a, float b) {
    float aRad = a * DEG_TO_RAD;
    float bRad = b * DEG_TO_RAD;
    float x = cosf(aRad) + cosf(bRad);
    float y = sinf(aRad) + sinf(bRad);
    float ang = atan2f(y, x) * 180.0f / PI;
    return wrap360(ang);
}


bool fsm::findTrough(int minIdx, int &leftIdx, int &rightIdx) {
    if (minIdx < 0 || minIdx >= scanCount) return false;
    float minVal = scanDistances[minIdx];
    if (minVal < US_MIN_CM || minVal > US_MAX_CM) return false;

    float threshold = minVal + TROUGH_BAND_CM;
    leftIdx = minIdx;
    rightIdx = minIdx;

    while (leftIdx - 1 >= 0) {
        float d = scanDistances[leftIdx - 1];
        if (d < US_MIN_CM || d > US_MAX_CM || d > threshold) break;
        leftIdx--;
    }
    while (rightIdx + 1 < scanCount) {
        float d = scanDistances[rightIdx + 1];
        if (d < US_MIN_CM || d > US_MAX_CM || d > threshold) break;
        rightIdx++;
    }
    return (rightIdx - leftIdx + 1) >= MIN_TROUGH_SAMPLES;
}

int fsm::findAllTroughs(WallTrough troughs[], int maxTroughs) {
    int count = 0;

    for (int i = 1; i < scanCount - 1; i++) {
        float d = scanDistances[i];
        if (d < US_MIN_CM || d > US_MAX_CM) continue;

        float dPrev = scanDistances[i - 1];
        float dNext = scanDistances[i + 1];

        if (dPrev < US_MIN_CM || dPrev > US_MAX_CM) continue;
        if (dNext < US_MIN_CM || dNext > US_MAX_CM) continue;

        // local minimum candidate
        if (d <= dPrev && d <= dNext) {
            int leftIdx, rightIdx;
            if (!findTrough(i, leftIdx, rightIdx)) continue;

            // avoid duplicates from multiple minima inside same trough
            bool overlaps = false;
            for (int k = 0; k < count; k++) {
                if (!(rightIdx < troughs[k].leftIdx || leftIdx > troughs[k].rightIdx)) {
                    overlaps = true;
                    // keep the deeper trough minimum if needed
                    if (d < troughs[k].minDistCm) {
                        troughs[k].leftIdx = leftIdx;
                        troughs[k].minIdx = i;
                        troughs[k].rightIdx = rightIdx;
                        troughs[k].centerHeading = circularMeanDeg(scanHeadings[leftIdx], scanHeadings[rightIdx]);
                        troughs[k].minDistCm = d;
                    }
                    break;
                }
            }
            if (overlaps) continue;

            if (count < maxTroughs) {
                troughs[count].leftIdx = leftIdx;
                troughs[count].minIdx = i;
                troughs[count].rightIdx = rightIdx;
                troughs[count].centerHeading = circularMeanDeg(scanHeadings[leftIdx], scanHeadings[rightIdx]);
                troughs[count].minDistCm = d;
                count++;
            }
        }
    }

    return count;
}

int fsm::findOppositePairs(const WallTrough troughs[], int troughCount,
                           OppositePair pairs[], int maxPairs) {
    int count = 0;

    for (int i = 0; i < troughCount; i++) {
        for (int j = i + 1; j < troughCount; j++) {
            float directSep = angleDiffDeg(troughs[i].centerHeading,
                                           troughs[j].centerHeading);

            // easier to think of it directly as "close to 180"
            float err180 = fabsf(directSep - 180.0f);

            if (err180 <= OPPOSITE_TOL_DEG) {
                if (count < maxPairs) {
                    pairs[count].a = i;
                    pairs[count].b = j;
                    pairs[count].headingSepDeg = directSep;
                    pairs[count].spanCm =
                        troughs[i].minDistCm + troughs[j].minDistCm;
                    count++;
                }
            }
        }
    }

    return count;
}

bool fsm::chooseLongWallTarget(const WallTrough troughs[], int troughCount,
                               float &targetHeading, float &targetDistCm) {
    OppositePair pairs[8];
    int pairCount = findOppositePairs(troughs, troughCount, pairs, 8);
    if (pairCount <= 0) return false;

    int bestPair = -1;
    float bestSpan = 1e9f;

    for (int i = 0; i < pairCount; i++) {
        if (pairs[i].spanCm < bestSpan) {
            bestSpan = pairs[i].spanCm;
            bestPair = i;
        }
    }
    if (bestPair < 0) return false;

    const WallTrough &t1 = troughs[pairs[bestPair].a];
    const WallTrough &t2 = troughs[pairs[bestPair].b];

    const WallTrough &chosen = (t1.minDistCm <= t2.minDistCm) ? t1 : t2;

    targetHeading = chosen.centerHeading;
    targetDistCm = chosen.minDistCm;
    return true;
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
        heading           = 0.0f;
        lastUpdateUs      = micros();
        scanCount         = 0;
        scanStartHeading  = 0.0f;
        scanTargetHeading = 0.0f;
        chosenMinDistCm   = -1.0f;
        globalMinIdx      = -1;
        returnInTolActive = false;
        resetUsFilter();
        lastSampleMs      = millis();
        state             = HOMING_SCAN;
        break;

    case HOMING_SCAN:
        updateHeading();
        {
            unsigned long now = millis();
            if (now - lastSampleMs >= (unsigned long)SCAN_SAMPLE_INTERVAL) {
                lastSampleMs = now;
                float usRaw = perception->getUltrasonicCm();
                if (usRaw < US_MIN_CM || usRaw > US_MAX_CM) usRaw = -1.0f;
                float usFilt = usMovingAverage(usRaw);

                if (scanCount < MAX_SCAN_SAMPLES) {
                    scanHeadings[scanCount]  = heading;
                    scanDistances[scanCount] = usFilt;
                    scanCount++;
                }
            }
        }
        motors->RotateCCW(SCAN_SPIN_SPEED);
        if (heading >= 360.0f) {
            motors->Stop(true);
            Serial.print("Scan done. ");
            Serial.print(scanCount);
            Serial.println(" samples");
            state = HOMING_ANALYSE;
        }
        break;

    case HOMING_ANALYSE:
{
    WallTrough troughs[8];
    int troughCount = findAllTroughs(troughs, 8);

    Serial.print("Found troughs: ");
    Serial.println(troughCount);

    for (int i = 0; i < troughCount; i++) {
        Serial.print("T");
        Serial.print(i);
        Serial.print(": min=");
        Serial.print(troughs[i].minDistCm, 1);
        Serial.print(" cm, center=");
        Serial.println(troughs[i].centerHeading, 1);
    }

    float targetHeading, targetDistCm;
    if (!chooseLongWallTarget(troughs, troughCount, targetHeading, targetDistCm)) {
        Serial.println("ERROR: could not identify long-wall pair, retrying");
        state = HOMING_IDLE;
        break;
    }

    scanTargetHeading = targetHeading;
    chosenMinDistCm = targetDistCm;

    Serial.print("Chosen long-wall target dist=");
    Serial.print(chosenMinDistCm, 1);
    Serial.print(" cm, heading=");
    Serial.println(scanTargetHeading, 1);

    returnInTolActive = false;
    state = HOMING_RETURN;
    }
    break;

    case HOMING_RETURN:
        updateHeading();
        {
            float errDeg = signedAngleErrorDeg(scanTargetHeading, heading);
            float absErr = fabsf(errDeg);
            unsigned long now = millis();

            if (absErr <= RETURN_STOP_TOL_DEG) {
                motors->Stop(true);
                if (!returnInTolActive) {
                    returnInTolActive = true;
                    returnInTolStart  = now;
                } else if (now - returnInTolStart >= RETURN_STABLE_MS) {
                    Serial.println("Facing wall — moving right");
                    state = HOMING_APPROACH_WALL;
                }
            } else {
                returnInTolActive = false;
                int spd = (absErr < RETURN_SLOW_BAND_DEG) ? RETURN_SPIN_SLOW : RETURN_SPIN_FAST;
                if (errDeg > 0.0f)
                    motors->RotateCCW(spd);
                else
                    motors->RotateCW(spd);
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
            Serial.println("Corner rammed — backing off to 86 mm");
            state = HOMING_BACK_OFF;
        } else {
            motors->MoveForward(RAM_SPEED);
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
                    int vy = (int)motors->wallFollowCorrection(87.0f);
                    motors->drive(-MOVE_SPEED, vy, (int)motors->headingCorrection());
                } else if (runCount >= 9) {
                    int vy = (int)motors->wallFollowCorrection(187.0f, true);
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
                if (runCount >= 9) {
                    int vy = (int)motors->wallFollowCorrection(187.0f, true);
                    motors->drive(MOVE_SPEED, vy, (int)motors->headingCorrection());
                } else {
                    motors->drive(MOVE_SPEED, 0, (int)motors->headingCorrection());
                }
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
                int vy = (int)motors->wallFollowCorrection(100.0f, true);
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
                int vy = (int)motors->wallFollowCorrection(100.0f, true);
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

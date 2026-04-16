#include "fsm.h"
#include <Arduino.h>
#include <math.h>

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   ROTATE_SPEED       = 150;
static const float US_SPIKE_THRESHOLD = 60000.0f;
static const int   US_WARMUP_READINGS = 5;
static const float RETURN_MIN_DEG     = 20.0f;
static const int   RETURN_EXTRA_MS    = 100;
static const float APPROACH_STOP_CM   = 10.0f;//change this for better score
static const int   APPROACH_SPEED     = 200;
static const float FORWARD_STOP_MM    = 100.0f;//and this one
static const int   FORWARD_SPEED      = 200;

static const int   MOVE_SPEED         = 200;
static const float REAR_STOP_MM       = 100.0f;//and this one
static const float FRONT_STOP_MM      = 100.0f;///and this one
static const float LEFT_STOP_MM       = 120.0f;
static const float LEFT_IGNORE_MM     = 110.0f;
static const int   LEFT_CONFIRM_N     = 20;
static const int   LEFT_RESET_N       = 10;
static const unsigned long STRAFE_TIME_MS = 600;//tune the strafe time between cuts

static const float SQUARE_DIFF            =  9.0f; // us_mm - ir_mm when square; calibrate this
static const float SQUARE_KP             =  25.0f;
static const float SQUARE_KI             =   0.05f;
static const float SQUARE_MAX_INT        = 200.0f;  // integral anti-windup clamp
static const float SQUARE_MAX_SPEED      =  80.0f;
static const float SQUARE_MIN_SPEED      =  30.0f;
static const float SQUARE_THRESHOLD_MM   =   3.0f;
static const unsigned long SQUARE_HOLD_MS = 3000;  // must stay within threshold for this long
static const float SQUARE_US_ALPHA       =   0.2f; // EMA smoothing for US in square-up

static const float WF_SETPOINT_CM = 9.0f;//and this one
static const float WF_KP          = 20.0f;
static const float WF_KI          =  0.5f;
static const float WF_MAX_INT     = 200.0f;

// ── Run wall-follow (US sensor, incrementing setpoint) ────────────────────────
static const float WF_RUN_START_MM      = 100.0f;  // initial setpoint: 10 cm
static const float WF_RUN_STRAFE_INC_MM = 100.0f;  // +10 cm per strafe
static const float WF_RUN_KP            =   1.0f;
static const float WF_RUN_KI            =   0.001f;
static const float WF_RUN_MAX_INT       = 200.0f;
// ─────────────────────────────────────────────────────────────────────────────

fsm::fsm(percepetion *perception, movement *motors)
    : perception(perception), motors(motors),
      state(HOMING_IDLE),
      heading(0.0f), lastUpdateUs(0),
      topCount(0), minUsDist(9999.0f), minUsHeading(0.0f),
      lastValidUs(-1.0f), usReadingCount(0), lastSampleMs(0),
      returnStartHeading(0.0f), returnExtraStart(-1), lastSampleUsMs(0),
      squareInRangeStart(-1), squareLastPrintMs(0), squareUsSmoothed(-1.0f),
      squareIntegral(0.0f), squareLastUs(0),
      wf_integral(0.0f), wf_last_us(0),
      wf_run_setpoint_mm(WF_RUN_START_MM), wf_run_integral(0.0f), wf_run_last_us(0),
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
                squareInRangeStart = -1;
                squareUsSmoothed   = -1.0f;
                squareIntegral     = 0.0f;
                squareLastUs       = micros();
                Serial.println("At front wall — squaring up");
                state = HOMING_SQUARE_UP;
            } else {
                int vy = wfCorrection();
                motors->drive(FORWARD_SPEED, vy, (int)motors->headingCorrection());
            }
        }
        break;

    case HOMING_SQUARE_UP:
    {
        // ── dt calculation ───────────────────────────────────────────────
        unsigned long nowUs = micros();
        float dt = constrain((nowUs - squareLastUs) / 1e6f, 0.0f, 0.1f);
        squareLastUs = nowUs;

        // ── sensor smoothing ─────────────────────────────────────────────
        float us_raw = perception->getUltrasonicCm() * 10.0f;
        if (us_raw > 0.0f) {
            squareUsSmoothed = (squareUsSmoothed < 0.0f) ? us_raw
                             : SQUARE_US_ALPHA * us_raw + (1.0f - SQUARE_US_ALPHA) * squareUsSmoothed;
        }
        float us_mm = squareUsSmoothed;
        float ir_mm = perception->getIRMedRight();
        float error = (us_mm - ir_mm) - SQUARE_DIFF;

        // ── integral with anti-windup clamping ───────────────────────────
        squareIntegral = constrain(squareIntegral + error * dt,
                                   -SQUARE_MAX_INT, SQUARE_MAX_INT);

        // reset integral on zero-crossing to prevent overshoot
        if ((error > 0.0f && squareIntegral < 0.0f) ||
            (error < 0.0f && squareIntegral > 0.0f)) {
            squareIntegral = 0.0f;
        }

        // ── PI output ────────────────────────────────────────────────────
        float output = SQUARE_KP * error + SQUARE_KI * squareIntegral;

        // ── debug print ──────────────────────────────────────────────────
        unsigned long now = millis();
        if (now - squareLastPrintMs >= 200) {
            squareLastPrintMs = now;
            Serial.print("SQ us="); Serial.print(us_mm, 1);
            Serial.print(" ir="); Serial.print(ir_mm, 1);
            Serial.print(" err="); Serial.print(error, 1);
            Serial.print(" int="); Serial.print(squareIntegral, 1);
            Serial.print(" out="); Serial.println(output, 1);
        }

        // ── hold timer — settled when within threshold ───────────────────
        if (fabsf(error) < SQUARE_THRESHOLD_MM) {
            if (squareInRangeStart < 0) squareInRangeStart = (long)now;
            if (now - (unsigned long)squareInRangeStart >= SQUARE_HOLD_MS) {
                motors->Stop(true);
                squareIntegral = 0.0f;
                Serial.println("Squared up — starting run");
                state = RUN_MOVE_DOWN;
                break;
            }
        } else {
            squareInRangeStart = -1;
        }

        // ── actuate with clamped speed ───────────────────────────────────
        if (fabsf(error) >= SQUARE_THRESHOLD_MM) {
            int speed = (int)constrain(fabsf(output), SQUARE_MIN_SPEED, SQUARE_MAX_SPEED);
            if (output > 0.0f) motors->RotateCW(speed);
            else               motors->RotateCCW(speed);
        } else {
            motors->Stop(true);  // hold still while confirming
        }
        break;
    }

    default:
        break;
    }
}

// ── Run wall-follow ───────────────────────────────────────────────────────────

void fsm::resetRunWf() {
    wf_run_integral = 0.0f;
    wf_run_last_us  = micros();
}

int fsm::runWfCorrection() {
    unsigned long now = micros();
    float dt = constrain((now - wf_run_last_us) / 1e6f, 0.0f, 0.1f);
    wf_run_last_us = now;
    float dist = perception->getUltrasonicCm() * 10.0f;  // cm → mm
    if (dist <= 0.0f) return 0;
    float error = dist - wf_run_setpoint_mm;
    wf_run_integral = constrain(wf_run_integral + error * dt, -WF_RUN_MAX_INT, WF_RUN_MAX_INT);
    return (int)constrain(-(WF_RUN_KP * error + WF_RUN_KI * wf_run_integral), -300, 300);
}

// ── Run ───────────────────────────────────────────────────────────────────────

void fsm::doRun() {
    switch (state) {

    case RUN_MOVE_DOWN:
        if (leftWallDetected()) {
            motors->Stop(true);
            Serial.println("Left wall — final move down");
            resetRunWf();
            state = RUN_FINAL_MOVE_DOWN;
            break;
        }
        {
            float rearMm = perception->getIRLongRear();
            if (rearMm > 0.0f && rearMm <= REAR_STOP_MM) {
                motors->Stop(true);
                Serial.print("Rear wall — strafing to ");
                Serial.print(wf_run_setpoint_mm + WF_RUN_STRAFE_INC_MM, 0);
                Serial.println(" mm");
                resetRunWf();
                state = RUN_STRAFE_LEFT_A;
            } else {
                motors->drive(-MOVE_SPEED, (int)motors->wallFollowCorrection(wf_run_setpoint_mm), (int)motors->headingCorrection());
            }
        }
        break;

    case RUN_STRAFE_LEFT_A:
        {
            float usMm = perception->getUltrasonicCm() * 10.0f;
            if (usMm > 0.0f && usMm >= wf_run_setpoint_mm + WF_RUN_STRAFE_INC_MM) {
                motors->Stop(true);
                wf_run_setpoint_mm += WF_RUN_STRAFE_INC_MM;
                resetRunWf();
                Serial.print("Strafe done — setpoint now ");
                Serial.print(wf_run_setpoint_mm, 0);
                Serial.println(" mm — move up");
                state = RUN_MOVE_UP;
            } else {
                motors->MoveLeft(MOVE_SPEED);
            }
        }
        break;

    case RUN_MOVE_UP:
        if (leftWallDetected()) {
            motors->Stop(true);
            Serial.println("Left wall — final move up");
            resetRunWf();
            state = RUN_FINAL_MOVE_UP;
            break;
        }
        {
            float frontMm = perception->getIRMedFront();
            if (frontMm > 0.0f && frontMm <= FRONT_STOP_MM) {
                motors->Stop(true);
                Serial.print("Front wall — strafing to ");
                Serial.print(wf_run_setpoint_mm + WF_RUN_STRAFE_INC_MM, 0);
                Serial.println(" mm");
                resetRunWf();
                state = RUN_STRAFE_LEFT_B;
            } else {
                motors->drive(MOVE_SPEED, (int)motors->wallFollowCorrection(wf_run_setpoint_mm), (int)motors->headingCorrection());
            }
        }
        break;

    case RUN_STRAFE_LEFT_B:
        {
            float usMm = perception->getUltrasonicCm() * 10.0f;
            if (usMm > 0.0f && usMm >= wf_run_setpoint_mm + WF_RUN_STRAFE_INC_MM) {
                motors->Stop(true);
                wf_run_setpoint_mm += WF_RUN_STRAFE_INC_MM;
                resetRunWf();
                Serial.print("Strafe done — setpoint now ");
                Serial.print(wf_run_setpoint_mm, 0);
                Serial.println(" mm — move down");
                state = RUN_MOVE_DOWN;
            } else {
                motors->MoveLeft(MOVE_SPEED);
            }
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

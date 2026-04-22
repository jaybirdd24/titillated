#include "fsm.h"
#include <Arduino.h>
#include <math.h>

// ── Tuning ────────────────────────────────────────────────────────────────────
static const int   ROTATE_SPEED       = 150;

// ── Step-scan tuning ──────────────────────────────────────────────────────────
static const int   SCAN_STEP_DEG      = 10;      // degrees per scan step
static const int   SCAN_N_SAMPLES     = 6;       // US readings per step
static const int   SCAN_SETTLE_MS     = 80;      // ms to wait after stopping before sampling
static const int   SCAN_INTERVAL_MS   = 40;      // ms between US readings within a step
static const float SCAN_SPIKE_CM      = 25.0f;   // reject if jump > this from last valid
static const int   SCAN_ROTATE_SPEED  = 100;     // slower rotation for heading accuracy
static const float FLAT_MIN_RANGE_CM  = 3.0f;    // ignore readings below this (spurious close)
static const float FLAT_MAX_RANGE_CM  = 200.0f;  // ignore readings above this (out of range)
static const float FLAT_SLOPE_THR     = 6.0f;    // cm — slope threshold for flat cluster
static const float RECT_SPACING_TOL   = 20.0f;   // degrees tolerance for 90° rectangle check

// ── Refine tuning ─────────────────────────────────────────────────────────────
static const float REFINE_DELTA_DEG   = 10.0f;   // dither ±this many degrees from bestHeading
static const float REFINE_EPS_CM      = 1.5f;    // converged when |dL - dR| < this
static const int   REFINE_MAX_ITER    = 5;        // max dither iterations
static const float REFINE_ADJ_DEG     = 3.0f;    // heading shift per iteration


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
static const unsigned long STRAFE_TIME_MS = 592;//tune the strafe time between cuts
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
      scanStep(0), scanSampleCount(0), scanSampleSum(0.0f),
      scanLastValid(-1.0f), scanSettleStart(0), scanSampleLast(0),
      bestHeading(-1.0f), bestRange(9999.0f),
      refinePhase(0), refineLeft(0.0f), refineRight(0.0f),
      refineIter(0), refineSettleStart(0),
      refineSampleCount(0), refineSampleSum(0.0f),
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
    for (int i = 0; i < SCAN_STEPS_MAX; i++) scanRange[i] = -1.0f;
}

fsm::~fsm() {}

// ── Heading (FSM-local integrator, kept for doRun heading display) ────────────

void fsm::updateHeading() {
    unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1e6f;
    lastUpdateUs = now;
    heading += perception->getGyroZ() * (180.0f / PI) * dt;
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
        motors->resetHeading();
        for (int i = 0; i < SCAN_STEPS_MAX; i++) scanRange[i] = -1.0f;
        scanStep        = 0;
        scanSampleCount = 0;
        scanSampleSum   = 0.0f;
        scanLastValid   = -1.0f;
        bestHeading     = -1.0f;
        bestRange       = 9999.0f;
        state           = HOMING_SCAN_ROTATE;
        break;

    case HOMING_SCAN_ROTATE:
        {
            float target = scanStep * (float)SCAN_STEP_DEG;
            if (motors->getHeading() >= target - 2.0f) {
                motors->Stop(true);
                scanSettleStart = millis();
                scanSampleCount = 0;
                scanSampleSum   = 0.0f;
                scanLastValid   = -1.0f;
                scanSampleLast  = millis();
                state = HOMING_SCAN_SETTLE;
            } else {
                motors->RotateCCW(SCAN_ROTATE_SPEED);
            }
        }
        break;

    case HOMING_SCAN_SETTLE:
        if (millis() - scanSettleStart >= (unsigned long)SCAN_SETTLE_MS) {
            state = HOMING_SCAN_SAMPLE;
        }
        break;

    case HOMING_SCAN_SAMPLE:
        {
            unsigned long now = millis();
            if (now - scanSampleLast >= (unsigned long)SCAN_INTERVAL_MS) {
                scanSampleLast = now;
                float r = perception->getUltrasonicCm();
                bool ok = (r > FLAT_MIN_RANGE_CM) && (r < FLAT_MAX_RANGE_CM) &&
                          (scanLastValid < 0.0f || fabsf(r - scanLastValid) < SCAN_SPIKE_CM);
                if (ok) {
                    scanSampleSum += r;
                    scanSampleCount++;
                    scanLastValid = r;
                }
            }
            // advance when enough valid samples or timeout (2× budget)
            unsigned long budget = (unsigned long)(SCAN_SETTLE_MS + SCAN_N_SAMPLES * SCAN_INTERVAL_MS * 2);
            if (scanSampleCount >= SCAN_N_SAMPLES || millis() - scanSettleStart >= budget) {
                scanRange[scanStep] = (scanSampleCount > 0)
                    ? scanSampleSum / scanSampleCount
                    : -1.0f;
                scanStep++;
                if (scanStep >= SCAN_STEPS_MAX) {
                    motors->Stop(true);
                    state = HOMING_SCAN_PROCESS;
                } else {
                    scanSampleCount = 0;
                    scanSampleSum   = 0.0f;
                    scanLastValid   = -1.0f;
                    state = HOMING_SCAN_ROTATE;
                }
            }
        }
        break;

    case HOMING_SCAN_PROCESS:
        {
            const int N = SCAN_STEPS_MAX;

            // 1. 3-point wrap-around moving average
            float smoothed[N];
            for (int i = 0; i < N; i++) {
                float r = scanRange[i];
                if (r < 0.0f) { smoothed[i] = -1.0f; continue; }
                int ip = (i - 1 + N) % N, in_ = (i + 1) % N;
                float rp = (scanRange[ip] > 0.0f) ? scanRange[ip] : r;
                float rn = (scanRange[in_] > 0.0f) ? scanRange[in_] : r;
                smoothed[i] = (rp + r + rn) / 3.0f;
            }

            // 2. Local slope S[i] = |smoothed[i+1] - smoothed[i-1]| (wrap-around)
            float slope[N];
            for (int i = 0; i < N; i++) {
                if (smoothed[i] < 0.0f) { slope[i] = 9999.0f; continue; }
                int ip = (i - 1 + N) % N, in_ = (i + 1) % N;
                float sp = (smoothed[ip] >= 0.0f) ? smoothed[ip] : smoothed[i];
                float sn = (smoothed[in_] >= 0.0f) ? smoothed[in_] : smoothed[i];
                slope[i] = fabsf(sn - sp);
            }

            // 3. Find contiguous low-slope clusters; track start/end for seam merge
            struct ClusterData {
                float headingCentre, rangeMean, slopeMean;
                int   widthSteps, startStep, endStep;
            };
            const int MAX_CL = 8;
            ClusterData cl[MAX_CL];
            int nCl = 0;

            bool  inCluster = false;
            float clSumH = 0.0f, clSumR = 0.0f, clSumS = 0.0f;
            int   clCnt = 0, clStart = 0;

            for (int i = 0; i <= N; i++) {
                bool flat = (i < N) && (slope[i] < FLAT_SLOPE_THR) && (smoothed[i] > 0.0f);
                if (flat) {
                    if (!inCluster) { inCluster = true; clStart = i; }
                    clSumH += i * (float)SCAN_STEP_DEG;
                    clSumR += smoothed[i];
                    clSumS += slope[i];
                    clCnt++;
                } else if (inCluster) {
                    inCluster = false;
                    if (nCl < MAX_CL) {
                        cl[nCl].headingCentre = clSumH / clCnt;
                        cl[nCl].rangeMean     = clSumR / clCnt;
                        cl[nCl].slopeMean     = clSumS / clCnt;
                        cl[nCl].widthSteps    = clCnt;
                        cl[nCl].startStep     = clStart;
                        cl[nCl].endStep       = i - 1;
                        nCl++;
                    }
                    clSumH = clSumR = clSumS = 0.0f;
                    clCnt = 0;
                }
            }

            // 4. Merge first and last cluster if they straddle the 0°/360° seam
            if (nCl >= 2 && cl[0].startStep == 0 && cl[nCl - 1].endStep == N - 1) {
                float wA = (float)cl[0].widthSteps;
                float wB = (float)cl[nCl - 1].widthSteps;
                float total = wA + wB;
                // Wrap last cluster's centre below 0° so the weighted mean is correct
                float hB = cl[nCl - 1].headingCentre - 360.0f;
                float merged = (cl[0].headingCentre * wA + hB * wB) / total;
                if (merged < 0.0f) merged += 360.0f;
                cl[0].headingCentre = merged;
                cl[0].rangeMean     = (cl[0].rangeMean * wA + cl[nCl - 1].rangeMean * wB) / total;
                cl[0].slopeMean     = (cl[0].slopeMean * wA + cl[nCl - 1].slopeMean * wB) / total;
                cl[0].widthSteps    = (int)total;
                nCl--;
                Serial.println("  seam merge applied");
            }

            Serial.print("Scan: "); Serial.print(nCl); Serial.println(" clusters");
            for (int i = 0; i < nCl; i++) {
                Serial.print("  cl["); Serial.print(i); Serial.print("] h=");
                Serial.print(cl[i].headingCentre, 1);
                Serial.print(" r="); Serial.print(cl[i].rangeMean, 1);
                Serial.print(" w="); Serial.print(cl[i].widthSteps);
                Serial.print(" s="); Serial.println(cl[i].slopeMean, 2);
            }

            // 5. Cluster score: lower = better (closer wall + wider flat region)
            //    score = rangeMean / widthSteps
            bestHeading    = -1.0f;
            bestRange      = 9999.0f;
            float bestScore = 9999.0f;

            // 5a. Rectangle geometry: prefer a 90°-spaced pair
            for (int a = 0; a < nCl; a++) {
                for (int b = a + 1; b < nCl; b++) {
                    float sp = fabsf(cl[b].headingCentre - cl[a].headingCentre);
                    if (sp > 180.0f) sp = 360.0f - sp;
                    if (fabsf(sp - 90.0f) <= RECT_SPACING_TOL) {
                        float sA = cl[a].rangeMean / cl[a].widthSteps;
                        float sB = cl[b].rangeMean / cl[b].widthSteps;
                        int   pick = (sA <= sB) ? a : b;
                        float s    = (sA <= sB) ? sA : sB;
                        if (s < bestScore) {
                            bestScore   = s;
                            bestRange   = cl[pick].rangeMean;
                            bestHeading = cl[pick].headingCentre;
                        }
                    }
                }
            }

            // 5b. Fallback: best-scoring flat cluster (no 90° pair found)
            if (bestHeading < 0.0f) {
                for (int a = 0; a < nCl; a++) {
                    float s = cl[a].rangeMean / cl[a].widthSteps;
                    if (s < bestScore) {
                        bestScore   = s;
                        bestRange   = cl[a].rangeMean;
                        bestHeading = cl[a].headingCentre;
                    }
                }
            }

            // 5c. Final fallback: raw minimum smoothed reading
            if (bestHeading < 0.0f) {
                for (int i = 0; i < N; i++) {
                    if (smoothed[i] > 0.0f && smoothed[i] < bestRange) {
                        bestRange   = smoothed[i];
                        bestHeading = i * (float)SCAN_STEP_DEG;
                    }
                }
            }

            Serial.print("Best h="); Serial.print(bestHeading, 1);
            Serial.print(" r="); Serial.println(bestRange, 1);

            refinePhase       = 0;
            refineIter        = 0;
            refineSampleCount = 0;
            refineSampleSum   = 0.0f;
            state = HOMING_APPROACH_WALL;
        }
        break;

    // case HOMING_SCAN_REFINE:
    //     {
    //         unsigned long now = millis();
    //         float curH = motors->getHeading();
    //         // Maximum time we're willing to wait for SCAN_N_SAMPLES valid readings
    //         static const unsigned long REFINE_SAMPLE_BUDGET_MS =
    //             (unsigned long)(SCAN_N_SAMPLES * SCAN_INTERVAL_MS * 3);

    //         if (refinePhase == 0) {
    //             // Rotate CW from ~360° to (bestHeading - REFINE_DELTA)
    //             // Heading is a continuous integrator; this works even when target goes negative
    //             float tgt = bestHeading - REFINE_DELTA_DEG;
    //             if (curH <= tgt + 3.0f) {
    //                 motors->Stop(true);
    //                 refineSettleStart = now;
    //                 refinePhase = 1;
    //             } else {
    //                 motors->RotateCW(ROTATE_SPEED);
    //             }

    //         } else if (refinePhase == 1) {
    //             // Settle after stopping at -delta position
    //             if (now - refineSettleStart >= (unsigned long)SCAN_SETTLE_MS) {
    //                 refineSampleCount = 0;
    //                 refineSampleSum   = 0.0f;
    //                 scanLastValid     = -1.0f;
    //                 scanSampleLast    = now;
    //                 refineSettleStart = now; // reused as sample-budget start
    //                 refinePhase = 2;
    //             }

    //         } else if (refinePhase == 2) {
    //             // Sample → refineLeft  (range at bestHeading − REFINE_DELTA)
    //             if (now - scanSampleLast >= (unsigned long)SCAN_INTERVAL_MS) {
    //                 scanSampleLast = now;
    //                 float r = perception->getUltrasonicCm();
    //                 bool ok = (r > FLAT_MIN_RANGE_CM) && (r < FLAT_MAX_RANGE_CM) &&
    //                           (scanLastValid < 0.0f || fabsf(r - scanLastValid) < SCAN_SPIKE_CM);
    //                 if (ok) { refineSampleSum += r; refineSampleCount++; scanLastValid = r; }
    //             }
    //             bool done = (refineSampleCount >= SCAN_N_SAMPLES) ||
    //                         (now - refineSettleStart >= REFINE_SAMPLE_BUDGET_MS);
    //             if (done) {
    //                 // Use partial mean on timeout; fall back to scan estimate if no valid reads
    //                 refineLeft        = (refineSampleCount > 0)
    //                                     ? refineSampleSum / refineSampleCount
    //                                     : bestRange;
    //                 refineSampleCount = 0;
    //                 refineSampleSum   = 0.0f;
    //                 scanLastValid     = -1.0f;
    //                 refinePhase = 3;
    //             }

    //         } else if (refinePhase == 3) {
    //             // Rotate CCW to (bestHeading + REFINE_DELTA)
    //             float tgt = bestHeading + REFINE_DELTA_DEG;
    //             if (curH >= tgt - 3.0f) {
    //                 motors->Stop(true);
    //                 refineSettleStart = now;
    //                 refinePhase = 4;
    //             } else {
    //                 motors->RotateCCW(SCAN_ROTATE_SPEED);
    //             }

    //         } else if (refinePhase == 4) {
    //             // Settle after stopping at +delta position
    //             if (now - refineSettleStart >= (unsigned long)SCAN_SETTLE_MS) {
    //                 refineSampleCount = 0;
    //                 refineSampleSum   = 0.0f;
    //                 scanLastValid     = -1.0f;
    //                 scanSampleLast    = now;
    //                 refineSettleStart = now;
    //                 refinePhase = 5;
    //             }

    //         } else if (refinePhase == 5) {
    //             // Sample → refineRight  (range at bestHeading + REFINE_DELTA)
    //             if (now - scanSampleLast >= (unsigned long)SCAN_INTERVAL_MS) {
    //                 scanSampleLast = now;
    //                 float r = perception->getUltrasonicCm();
    //                 bool ok = (r > FLAT_MIN_RANGE_CM) && (r < FLAT_MAX_RANGE_CM) &&
    //                           (scanLastValid < 0.0f || fabsf(r - scanLastValid) < SCAN_SPIKE_CM);
    //                 if (ok) { refineSampleSum += r; refineSampleCount++; scanLastValid = r; }
    //             }
    //             bool done = (refineSampleCount >= SCAN_N_SAMPLES) ||
    //                         (now - refineSettleStart >= REFINE_SAMPLE_BUDGET_MS);
    //             if (done) {
    //                 refineRight = (refineSampleCount > 0)
    //                               ? refineSampleSum / refineSampleCount
    //                               : bestRange;
    //                 refinePhase = 6;
    //             }

    //         } else if (refinePhase == 6) {
    //             // e = refineLeft − refineRight
    //             // Convention (US sensor on the RIGHT, RotateCCW increases heading):
    //             //   refineLeft  = range at (bestHeading − delta) — slightly CW of best
    //             //   refineRight = range at (bestHeading + delta) — slightly CCW of best
    //             // Minimum range = perpendicular to wall.
    //             // e > 0  →  right side closer  →  perp is CCW of current best  →  bestHeading += adj
    //             // e < 0  →  left side closer   →  perp is CW  of current best  →  bestHeading -= adj
    //             //
    //             // Lab check: place robot ~5° off perpendicular from a flat wall, run homing,
    //             // watch Serial output. Each iteration bestHeading should step toward the wall
    //             // normal. If it steps away, negate the adj below.
    //             float e = refineLeft - refineRight;
    //             Serial.print("Refine "); Serial.print(refineIter);
    //             Serial.print(" L="); Serial.print(refineLeft, 1);
    //             Serial.print(" R="); Serial.print(refineRight, 1);
    //             Serial.print(" e="); Serial.println(e, 2);

    //             bool converged = (fabsf(e) < REFINE_EPS_CM) || (refineIter >= REFINE_MAX_ITER - 1);
    //             if (!converged) {
    //                 bestHeading += (e > 0.0f) ? REFINE_ADJ_DEG : -REFINE_ADJ_DEG;
    //                 refineIter++;
    //                 refineSampleCount = 0;
    //                 refineSampleSum   = 0.0f;
    //                 refinePhase = 0;
    //             } else {
    //                 refinePhase = 7;
    //             }

    //         } else if (refinePhase == 7) {
    //             // Rotate CW to final bestHeading
    //             if (curH <= bestHeading + 3.0f) {
    //                 motors->Stop(true);
    //                 refineSettleStart = now;
    //                 refinePhase = 8;
    //             } else {
    //                 motors->RotateCW(ROTATE_SPEED);
    //             }

    //         } else { // refinePhase == 8
    //             if (now - refineSettleStart >= (unsigned long)SCAN_SETTLE_MS) {
    //                 Serial.print("Aligned h=");
    //                 Serial.print(motors->getHeading(), 1);
    //                 Serial.println(" — approaching wall");
    //                 state = HOMING_APPROACH_WALL;
    //             }
    //         }
    //     }
    //     break;

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
                int vy = (int)motors->wallFollowCorrection(93.0f, WALL_IR_LEFT);
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
                int vy = (int)motors->wallFollowCorrection(93.0f, WALL_IR_LEFT);
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

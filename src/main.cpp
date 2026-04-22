#include <Arduino.h>
#include <math.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine(&perception, &motors);

// ============================================================
// Settings
// ============================================================
static const int   LOG_INTERVAL_MS          = 50;     // 20 Hz
static const int   SCAN_SPIN_SPEED          = 120;
static const int   RETURN_SPIN_FAST         = 75;
static const int   RETURN_SPIN_SLOW         = 60;
static const float RETURN_SLOW_BAND_DEG     = 10.0f;
static const float RETURN_STOP_TOL_DEG      = 2.0f;
static const unsigned long RETURN_STABLE_MS = 250;

static const float US_MIN_CM                = 2.0f;
static const float US_MAX_CM                = 130.0f;

static const int   FILTER_SIZE              = 5;
static const float TROUGH_BAND_CM           = 1.5f;
static const int   MIN_TROUGH_SAMPLES       = 3;

// Keep this modest for Mega SRAM
static const int   MAX_SCAN_SAMPLES         = 180;

// ============================================================
// Types
// ============================================================
enum TestState {
    TEST_INIT,
    TEST_SCANNING,
    TEST_ANALYSE_SCAN,
    TEST_RETURN_TO_TARGET,
    TEST_DONE
};

// ============================================================
// Globals
// ============================================================
TestState testState = TEST_INIT;

unsigned long lastLogMs = 0;
unsigned long inTolStartMs = 0;
bool inTolTimerActive = false;

float usFilterBuf[FILTER_SIZE];
int   usFilterCount = 0;
int   usFilterHead = 0;

// lean scan storage
float headingRawStore[MAX_SCAN_SAMPLES];
float usFiltStore[MAX_SCAN_SAMPLES];
int   scanCount = 0;

float scanStartHeadingRaw = 0.0f;
float scanTargetHeadingRaw = 0.0f;
float chosenMinDistanceCm = -1.0f;

int globalMinIndex = -1;
int troughLeftIndex = -1;
int troughRightIndex = -1;

// ============================================================
// Helpers
// ============================================================
float wrap360(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

bool isValidUs(float us) {
    return (us >= US_MIN_CM && us <= US_MAX_CM);
}

void stopRobot() {
    motors.Stop(true);
}

void resetFilter() {
    usFilterCount = 0;
    usFilterHead = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        usFilterBuf[i] = -1.0f;
    }
}

float movingAverageUltrasonic(float us) {
    usFilterBuf[usFilterHead] = us;
    usFilterHead = (usFilterHead + 1) % FILTER_SIZE;

    if (usFilterCount < FILTER_SIZE) {
        usFilterCount++;
    }

    float sum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < usFilterCount; i++) {
        if (isValidUs(usFilterBuf[i])) {
            sum += usFilterBuf[i];
            validCount++;
        }
    }

    if (validCount == 0) {
        return -1.0f;
    }

    return sum / (float)validCount;
}

void resetScan() {
    scanCount = 0;
    globalMinIndex = -1;
    troughLeftIndex = -1;
    troughRightIndex = -1;
    scanTargetHeadingRaw = 0.0f;
    chosenMinDistanceCm = -1.0f;
    inTolTimerActive = false;
    resetFilter();
}

bool addScanSample(float rawHeading, float usFilt) {
    if (scanCount >= MAX_SCAN_SAMPLES) {
        return false;
    }

    headingRawStore[scanCount] = rawHeading;
    usFiltStore[scanCount] = usFilt;
    scanCount++;
    return true;
}

float scanProgressDeg(float currentRawHeading) {
    return currentRawHeading - scanStartHeadingRaw;
}

bool findGlobalMinimumIndex(int &minIdx) {
    minIdx = -1;
    float best = 1e9f;

    for (int i = 0; i < scanCount; i++) {
        float d = usFiltStore[i];
        if (!isValidUs(d)) {
            continue;
        }

        if (d < best) {
            best = d;
            minIdx = i;
        }
    }

    return (minIdx >= 0);
}

bool findStableTroughAroundMinimum(int minIdx, int &leftIdx, int &rightIdx) {
    if (minIdx < 0 || minIdx >= scanCount) {
        return false;
    }

    float minVal = usFiltStore[minIdx];
    if (!isValidUs(minVal)) {
        return false;
    }

    float threshold = minVal + TROUGH_BAND_CM;

    leftIdx = minIdx;
    rightIdx = minIdx;

    while (leftIdx - 1 >= 0) {
        float d = usFiltStore[leftIdx - 1];
        if (!isValidUs(d) || d > threshold) {
            break;
        }
        leftIdx--;
    }

    while (rightIdx + 1 < scanCount) {
        float d = usFiltStore[rightIdx + 1];
        if (!isValidUs(d) || d > threshold) {
            break;
        }
        rightIdx++;
    }

    int width = rightIdx - leftIdx + 1;
    return (width >= MIN_TROUGH_SAMPLES);
}

float computeTroughCenterHeadingRaw(int leftIdx, int rightIdx) {
    return 0.5f * (headingRawStore[leftIdx] + headingRawStore[rightIdx]);
}

void printAnalysis() {
    Serial.println("# ==================================================");
    Serial.println("# SCAN ANALYSIS");
    Serial.print("# samples = ");
    Serial.println(scanCount);

    Serial.print("# global min idx = ");
    Serial.println(globalMinIndex);
    Serial.print("# global min raw heading = ");
    Serial.println(headingRawStore[globalMinIndex], 3);
    Serial.print("# global min wrapped heading = ");
    Serial.println(wrap360(headingRawStore[globalMinIndex]), 3);
    Serial.print("# global min distance cm = ");
    Serial.println(usFiltStore[globalMinIndex], 3);

    Serial.print("# trough left wrapped heading = ");
    Serial.println(wrap360(headingRawStore[troughLeftIndex]), 3);
    Serial.print("# trough right wrapped heading = ");
    Serial.println(wrap360(headingRawStore[troughRightIndex]), 3);

    Serial.print("# target raw heading = ");
    Serial.println(scanTargetHeadingRaw, 3);
    Serial.print("# target wrapped heading = ");
    Serial.println(wrap360(scanTargetHeadingRaw), 3);
    Serial.println("# ==================================================");
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("# === SETUP START ===");

    Serial.println("# Initializing perception...");
    if (!perception.init()) {
        Serial.println("# ERROR: perception init failed");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("# Enabling motors...");
    motors.enable();
    stopRobot();
    Serial.println("# Motors stopped");

    unsigned long settleStart = millis();
    while (millis() - settleStart < 1500) {
        perception.update();
        delay(5);
    }

    motors.resetHeading();
    resetScan();

    Serial.println("# auto-start full scan");
    Serial.println("t_ms,raw_heading_deg,wrapped_heading_deg,us_cm_raw,us_cm_filt,event");
}

// ============================================================
// Loop
// ============================================================
void loop() {
    perception.update();

    unsigned long now = millis();
    if (now - lastLogMs < LOG_INTERVAL_MS) {
        delay(1);
        return;
    }
    lastLogMs = now;

    float rawHeading = motors.getHeading();
    float wrappedHeading = wrap360(rawHeading);

    float usRaw = perception.getUltrasonicCm();
    if (!isValidUs(usRaw)) {
        usRaw = -1.0f;
    }

    float usFilt = movingAverageUltrasonic(usRaw);
    int event = 0;

    switch (testState) {

        case TEST_INIT: {
            stopRobot();
            delay(200);

            resetScan();
            scanStartHeadingRaw = motors.getHeading();
            Serial.print("# scan start raw heading = ");
            Serial.println(scanStartHeadingRaw, 3);

            testState = TEST_SCANNING;
            break;
        }

        case TEST_SCANNING: {
            motors.RotateCCW(SCAN_SPIN_SPEED);

            if (!addScanSample(rawHeading, usFilt)) {
                stopRobot();
                Serial.println("# ERROR: scan buffer full");
                testState = TEST_DONE;
                break;
            }

            float progress = scanProgressDeg(rawHeading);
            if (progress >= 360.0f) {
                stopRobot();
                Serial.println("# full 360 scan complete");
                testState = TEST_ANALYSE_SCAN;
            }
            break;
        }

        case TEST_ANALYSE_SCAN: {
            if (!findGlobalMinimumIndex(globalMinIndex)) {
                Serial.println("# ERROR: no valid minimum found");
                testState = TEST_DONE;
                break;
            }

            chosenMinDistanceCm = usFiltStore[globalMinIndex];

            if (!findStableTroughAroundMinimum(globalMinIndex, troughLeftIndex, troughRightIndex)) {
                troughLeftIndex = globalMinIndex;
                troughRightIndex = globalMinIndex;
            }

            scanTargetHeadingRaw = computeTroughCenterHeadingRaw(troughLeftIndex, troughRightIndex);

            printAnalysis();
            Serial.println("# rotating back to trough centre");
            testState = TEST_RETURN_TO_TARGET;
            break;
        }

        case TEST_RETURN_TO_TARGET: {
            float errDeg = scanTargetHeadingRaw - rawHeading;
            float absErr = fabs(errDeg);

            if (absErr <= RETURN_STOP_TOL_DEG) {
                stopRobot();

                if (!inTolTimerActive) {
                    inTolTimerActive = true;
                    inTolStartMs = now;
                } else if (now - inTolStartMs >= RETURN_STABLE_MS) {
                    event = 4;
                    Serial.println("# aligned to target heading");
                    Serial.print("# final raw heading = ");
                    Serial.println(rawHeading, 3);
                    Serial.print("# final wrapped heading = ");
                    Serial.println(wrappedHeading, 3);
                    Serial.print("# target raw heading = ");
                    Serial.println(scanTargetHeadingRaw, 3);
                    Serial.print("# target wrapped heading = ");
                    Serial.println(wrap360(scanTargetHeadingRaw), 3);
                    Serial.print("# chosen min distance cm = ");
                    Serial.println(chosenMinDistanceCm, 3);
                    testState = TEST_DONE;
                }
            } else {
                inTolTimerActive = false;

                int cmd = (absErr < RETURN_SLOW_BAND_DEG) ? RETURN_SPIN_SLOW : RETURN_SPIN_FAST;

                if (errDeg > 0.0f) {
                    motors.RotateCCW(cmd);
                } else {
                    motors.RotateCW(cmd);
                }
            }
            break;
        }

        case TEST_DONE:
        default: {
            stopRobot();
            break;
        }
    }

    if (testState == TEST_RETURN_TO_TARGET || testState == TEST_DONE) {
        if (fabs(scanTargetHeadingRaw - rawHeading) < 1.0f) {
            event = (event == 4) ? 4 : 3;
        }
    }

    Serial.print(now);
    Serial.print(",");
    Serial.print(rawHeading, 3);
    Serial.print(",");
    Serial.print(wrappedHeading, 3);
    Serial.print(",");
    Serial.print(usRaw, 3);
    Serial.print(",");
    Serial.print(usFilt, 3);
    Serial.print(",");
    Serial.println(event);

    delay(1);
}
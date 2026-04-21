#pragma once

#include "percepetion.h"
#include "movement.h"

enum RobotState {
    // ── Homing ────────────────────────────────────────────────────────────────
    HOMING_IDLE = 0,
    HOMING_SCAN,            // spin 360°, store heading + filtered US samples
    HOMING_ANALYSE,         // find trough centre heading from scan data
    HOMING_RETURN,          // rotate back to trough centre heading
    HOMING_APPROACH_WALL,   // move right until US < 15 cm
    HOMING_APPROACH_FWD,    // move forward until front IR < 150 mm
    HOMING_RAM_WALL,        // drive diagonally into corner to square up
    HOMING_BACK_OFF,        // strafe left until IR med right reads 86 mm
    // ── Run ───────────────────────────────────────────────────────────────────
    RUN_MOVE_DOWN,
    RUN_STRAFE_LEFT_A,
    RUN_MOVE_UP,
    RUN_STRAFE_LEFT_B,
    RUN_FINAL_MOVE_DOWN,
    RUN_FINAL_MOVE_UP,
    // ── Done ──────────────────────────────────────────────────────────────────
    STATE_DONE
};

class fsm
{
public:
    fsm(percepetion *perception, movement *motors);
    ~fsm();

    void        fsmUpdate();
    RobotState  getState()   const { return state; }
    float       getHeading() const { return heading; }

private:
    percepetion  *perception;
    movement     *motors;
    RobotState    state;

    // ── Heading integration ───────────────────────────────────────────────────
    float         heading;
    unsigned long lastUpdateUs;
    void          updateHeading();

    // ── Scan state ────────────────────────────────────────────────────────────
    static const int MAX_SCAN_SAMPLES = 180;
    static const int US_FILTER_SIZE   = 5;

    float  scanHeadings[MAX_SCAN_SAMPLES];   // raw heading at each sample
    float  scanDistances[MAX_SCAN_SAMPLES];  // filtered US at each sample
    int    scanCount;
    float  scanStartHeading;

    // US moving-average filter
    float  usFilterBuf[US_FILTER_SIZE];
    int    usFilterCount;
    int    usFilterHead;
    float  usMovingAverage(float us);
    void   resetUsFilter();

    // Scan analysis results
    float  scanTargetHeading;    // raw heading to return to
    float  chosenMinDistCm;      // distance at global minimum
    int    globalMinIdx;

    bool findGlobalMinIndex(int &minIdx);
    bool findTrough(int minIdx, int &leftIdx, int &rightIdx);

    // Return state
    unsigned long returnInTolStart;
    bool          returnInTolActive;

    unsigned long lastSampleMs;

    // ── Square-up state ───────────────────────────────────────────────────────
    long          squareInRangeStart;
    unsigned long squareLastPrintMs;
    float         squareUsSmoothed;
    float         squareIntegral;
    float         squarePrevError;
    unsigned long squareLastUs;

    // ── Ram / back-off state ────────────────────────────────────────────────
    unsigned long ramStartMs;
    long          backOffInRangeStart;

    // ── Wall-follow state (APPROACH_FWD) ─────────────────────────────────────
    float         wf_integral;
    unsigned long wf_last_us;
    int           wfCorrection();

    // ── Run state ─────────────────────────────────────────────────────────────
    unsigned long strafeStartMs;
    float         runHeading;   // saved heading at run start — kept for all passes
    float         runWfSetpoint; // IR distance (mm) captured after square-up for wall follow

    bool          leftWallSeen;
    int           leftNonIgnoreCount;
    int           leftIgnoreCount;
    unsigned long leftLastCountedMs;
    bool          firstRun;
    int           runCount;

    bool leftWallDetected();

    // ── FSM phases ────────────────────────────────────────────────────────────
    void doHoming();
    void doRun();
};

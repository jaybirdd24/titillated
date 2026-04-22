#pragma once

#include "percepetion.h"
#include "movement.h"

enum RobotState {
    // ── Homing ────────────────────────────────────────────────────────────────
    HOMING_IDLE = 0,
    HOMING_SCAN_ROTATE,     // step-scan: rotate CCW to next 10° increment
    HOMING_SCAN_SETTLE,     // step-scan: wait for vibration to damp
    HOMING_SCAN_SAMPLE,     // step-scan: collect US readings at this angle
    HOMING_SCAN_PROCESS,    // step-scan: cluster analysis, pick wall-normal heading
    HOMING_SCAN_REFINE,     // step-scan: dither ±delta, fine-tune bestHeading
    HOMING_APPROACH_WALL,   // move right until US < 15 cm
    HOMING_APPROACH_FWD,    // move forward until front IR < 150 mm
    HOMING_RAM_WALL,        // drive into right wall to physically square up
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

    // ── Step-scan state ───────────────────────────────────────────────────────
    static const int SCAN_STEPS_MAX = 36;   // 10 ° resolution → 360 / 10 = 36 steps
    float         scanRange[SCAN_STEPS_MAX]; // mean US range (cm) per step; -1 = invalid
    int           scanStep;                  // current step index
    int           scanSampleCount;           // valid samples collected at this step
    float         scanSampleSum;             // running sum for averaging
    float         scanLastValid;             // for per-step spike rejection
    unsigned long scanSettleStart;           // when we stopped rotating (settle timer)
    unsigned long scanSampleLast;            // last time a US sample was taken

    // Result of HOMING_SCAN_PROCESS
    float         bestHeading;               // chosen wall-normal heading (degrees)
    float         bestRange;                 // mean US range at bestHeading (cm)

    // Refine sub-state (HOMING_SCAN_REFINE)
    int           refinePhase;
    float         refineLeft;                // range at bestHeading - REFINE_DELTA
    float         refineRight;               // range at bestHeading + REFINE_DELTA
    int           refineIter;
    unsigned long refineSettleStart;
    int           refineSampleCount;
    float         refineSampleSum;

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

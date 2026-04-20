#pragma once

#include "percepetion.h"
#include "movement.h"

enum RobotState {
    // ── Homing ────────────────────────────────────────────────────────────────
    HOMING_IDLE = 0,
    HOMING_SCAN,            // spin 360°, record top-N closest US readings
    HOMING_RETURN,          // rotate CW back to averaged closest heading
    HOMING_APPROACH_WALL,   // move right until US < 15 cm
    HOMING_APPROACH_FWD,    // move forward until front IR < 150 mm
    HOMING_SET_DISTANCE,    // strafe to precise wall distance where SQUARE_DIFF is valid
    HOMING_SQUARE_UP,       // rotate until both right sensors match calibrated diff
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

    // ── Homing state ─────────────────────────────────────────────────────────
    static const int TOP_N = 10;
    float  topDist[TOP_N];
    float  topHead[TOP_N];
    int    topCount;
    float  minUsDist;
    float  minUsHeading;
    float  lastValidUs;
    int    usReadingCount;
    unsigned long lastSampleMs;
    float  returnStartHeading;
    long   returnExtraStart;
    unsigned long lastSampleUsMs;

    void insertTopN(float dist, float head);
    float avgTopHeading();
    float avgTopDist();

    // ── Square-up state ───────────────────────────────────────────────────────
    long          squareInRangeStart;
    unsigned long squareLastPrintMs;
    float         squareUsSmoothed;
    float         squareIntegral;
    float         squarePrevError;
    unsigned long squareLastUs;

    // ── Set-distance state ──────────────────────────────────────────────────
    long          setDistInRangeStart;

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

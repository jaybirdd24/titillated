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
    int           squareConfirmCount;
    unsigned long squareLastPrintMs;

    // ── Run state ─────────────────────────────────────────────────────────────
    unsigned long strafeStart;
    bool          leftWallSeen;
    int           leftNonIgnoreCount;
    int           leftIgnoreCount;
    unsigned long leftLastCountedMs;

    bool leftWallDetected();

    // ── FSM phases ────────────────────────────────────────────────────────────
    void doHoming();
    void doRun();
};

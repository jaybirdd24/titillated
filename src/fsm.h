#pragma once

#include "percepetion.h"
#include "movement.h"
#include "pose_estimator.h"

enum RobotState {
    // ── Homing ────────────────────────────────────────────────────────────────
    HOMING_IDLE = 0,
    HOMING_SCAN,            // spin 360°, record top-N closest US readings
    HOMING_RETURN,          // rotate CW back to averaged closest heading
    HOMING_APPROACH_WALL,   // move right until US < 15 cm
    HOMING_APPROACH_FWD,    // move forward until front IR < 150 mm
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
    fsm(percepetion *perception, movement *motors, pose_estimator *estimator);
    ~fsm();

    void        fsmUpdate();
    RobotState  getState()   const { return state; }
    float       getHeading() const;
    Pose2D      getPose()    const;

private:
    percepetion  *perception;
    movement     *motors;
    pose_estimator *estimator;
    RobotState    state;

    // ── Homing turn progress ──────────────────────────────────────────────────
    float         scanAccumulatedDeg;
    float         returnAccumulatedDeg;
    float         previousHeadingDeg;
    bool          previousHeadingValid;
    float         headingDeltaDeg();

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
    long   returnExtraStart;

    void insertTopN(float dist, float head);
    float avgTopHeading();
    float avgTopDist();
    void initializePoseFromHoming();

    // ── Run state ─────────────────────────────────────────────────────────────
    float         strafeTargetX;
    bool          leftWallSeen;
    int           leftNonIgnoreCount;
    int           leftIgnoreCount;
    unsigned long leftLastCountedMs;

    bool leftWallDetected();

    // ── FSM phases ────────────────────────────────────────────────────────────
    void doHoming();
    void doRun();
};

#include <Arduino.h>
#include <math.h>
#include "percepetion.h"
#include "movement.h"

// ── Square-up PI tuning (mirrors fsm.cpp) ────────────────────────────────────
static const float SQUARE_DIFF          =  9.0f;
static const float SQUARE_KP           =  25.0f;
static const float SQUARE_KI           =   0.05f;
static const float SQUARE_MAX_INT      = 200.0f;
static const float SQUARE_MAX_SPEED    =  80.0f;
static const float SQUARE_MIN_SPEED    =  30.0f;
static const float SQUARE_THRESHOLD_MM =   3.0f;
static const unsigned long SQUARE_HOLD_MS = 3000;
static const float SQUARE_US_ALPHA     =   0.2f;

// ── State ────────────────────────────────────────────────────────────────────
static percepetion perception;
static movement    motors(&perception);

static float         usSmoothed     = -1.0f;
static float         integral       =  0.0f;
static unsigned long lastUs         =  0;
static long          inRangeStart   = -1;
static unsigned long lastPrintMs    =  0;
static bool          done           = false;

void setup() {
    Serial.begin(115200);
    perception.init();
    motors.enable();
    delay(2000);

    usSmoothed   = -1.0f;
    integral     =  0.0f;
    lastUs       = micros();
    inRangeStart = -1;
    done         = false;

    Serial.println("SQUARE TEST START");
}

void loop() {
    perception.update();

    if (done) {
        motors.Stop(true);
        return;
    }

    // ── dt ───────────────────────────────────────────────────────────────────
    unsigned long nowUs = micros();
    float dt = constrain((nowUs - lastUs) / 1e6f, 0.0f, 0.1f);
    lastUs = nowUs;

    // ── sensor smoothing ─────────────────────────────────────────────────────
    float us_raw = perception.getUltrasonicCm() * 10.0f;
    if (us_raw > 0.0f) {
        usSmoothed = (usSmoothed < 0.0f) ? us_raw
                   : SQUARE_US_ALPHA * us_raw + (1.0f - SQUARE_US_ALPHA) * usSmoothed;
    }
    float us_mm = usSmoothed;
    float ir_mm = perception.getIRMedRight();
    float error = (us_mm - ir_mm) - SQUARE_DIFF;

    // ── integral with anti-windup clamping ───────────────────────────────────
    integral = constrain(integral + error * dt, -SQUARE_MAX_INT, SQUARE_MAX_INT);

    // reset integral on zero-crossing to prevent overshoot
    if ((error > 0.0f && integral < 0.0f) ||
        (error < 0.0f && integral > 0.0f)) {
        integral = 0.0f;
    }

    // ── PI output ────────────────────────────────────────────────────────────
    float output = SQUARE_KP * error + SQUARE_KI * integral;

    // ── debug print ──────────────────────────────────────────────────────────
    unsigned long now = millis();
    if (now - lastPrintMs >= 200) {
        lastPrintMs = now;
        Serial.print("SQ us=");  Serial.print(us_mm, 1);
        Serial.print(" ir=");    Serial.print(ir_mm, 1);
        Serial.print(" err=");   Serial.print(error, 1);
        Serial.print(" int=");   Serial.print(integral, 1);
        Serial.print(" out=");   Serial.println(output, 1);
    }

    // ── hold timer ───────────────────────────────────────────────────────────
    if (fabsf(error) < SQUARE_THRESHOLD_MM) {
        if (inRangeStart < 0) inRangeStart = (long)now;
        if (now - (unsigned long)inRangeStart >= SQUARE_HOLD_MS) {
            motors.Stop(true);
            integral = 0.0f;
            Serial.println("SQUARED UP — DONE");
            done = true;
            return;
        }
    } else {
        inRangeStart = -1;
    }

    // ── actuate with clamped speed ───────────────────────────────────────────
    if (fabsf(error) >= SQUARE_THRESHOLD_MM) {
        int speed = (int)constrain(fabsf(output), SQUARE_MIN_SPEED, SQUARE_MAX_SPEED);
        if (output > 0.0f) motors.RotateCW(speed);
        else               motors.RotateCCW(speed);
    } else {
        motors.Stop(true);
    }
}

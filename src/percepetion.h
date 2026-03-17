#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// ── Pin assignments (adjust to match your wiring) ──────────────────
// IR sensors — analog pins
constexpr uint8_t PIN_IR_LONG_FRONT  = A1;   // 2Y0A21, 100-800 mm
constexpr uint8_t PIN_IR_LONG_LEFT   = A2;   // 2Y0A21, 100-800 mm
constexpr uint8_t PIN_IR_MED_RIGHT   = A3;   // 2D120X / 2Y0A41SK, 40-300 mm
constexpr uint8_t PIN_IR_MED_REAR    = A4;   // 2D120X / 2Y0A41SK, 40-300 mm

// Ultrasonic HC-SR04
constexpr uint8_t PIN_US_TRIG = 48;
constexpr uint8_t PIN_US_ECHO = 49;
constexpr unsigned long US_MAX_PULSE_US = 23200;  // ~400 cm timeout

// Battery voltage divider
constexpr uint8_t PIN_BATTERY = A0;

// ── Smoothing ──────────────────────────────────────────────────────
constexpr uint8_t IR_FILTER_SAMPLES = 5;  // rolling-average window size

class percepetion
{
private:
    // ── IMU ────────────────────────────────────────────────────────
    Adafruit_BNO08x   bno08x;
    sh2_SensorValue_t  sensorValue;

    // ── IR sensor raw ring buffers (for smoothing) ─────────────────
    int  irLongFrontBuf[IR_FILTER_SAMPLES];
    int  irLongLeftBuf [IR_FILTER_SAMPLES];
    int  irMedRightBuf [IR_FILTER_SAMPLES];
    int  irMedRearBuf  [IR_FILTER_SAMPLES];
    uint8_t irBufIdx;

    // ── Ultrasonic ────────────────────────────────────────────────
    float usDistanceCm;

    // ── Helpers ───────────────────────────────────────────────────
    void  readIR();
    void  readUltrasonic();
    int   averageBuf(const int *buf, uint8_t len) const;

    // Convert raw ADC (0-1023) to distance in mm.
    // NOTE: these are starting-point approximations — calibrate with
    //       measured data and replace with your own curve fit.
    float irLongRawToMm(int raw)  const;   // 2Y0A21
    float irMedRawToMm(int raw)   const;   // 2D120X / 2Y0A41SK

public:
    percepetion();
    ~percepetion();

    // Call once in setup(); returns false if IMU not found
    bool init();

    // Call every loop() iteration — polls all sensors
    void update();

    // ── IMU getters ───────────────────────────────────────────────
    // Angular velocity around Z axis (rad/s), positive = CCW
    float getGyroZ();

    // ── IR getters (distance in mm, filtered) ─────────────────────
    float getIRLongFront();   // front-facing long-range
    float getIRLongLeft();    // left-facing  long-range
    float getIRMedRight();    // right-facing medium-range
    float getIRMedRear();     // rear-facing  medium-range

    // Raw ADC values (useful for calibration / serial logging)
    int   getIRLongFrontRaw();
    int   getIRLongLeftRaw();
    int   getIRMedRightRaw();
    int   getIRMedRearRaw();

    // ── Ultrasonic getter ─────────────────────────────────────────
    float getUltrasonicCm();  // distance in cm, 0 if no echo

    // ── Battery ───────────────────────────────────────────────────
    int   getBatteryRaw();            // raw ADC 0-1023
    float getBatteryVoltage();        // estimated voltage (V)
    bool  isBatteryLow();             // true when below safe threshold
};

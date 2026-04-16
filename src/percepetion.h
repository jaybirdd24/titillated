#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// ── Pin assignments (adjust to match your wiring) ──────────────────
// IR sensors — analog pins
constexpr uint8_t PIN_IR_MED_FRONT   = A10;  // 2D120X / 2Y0A41SK, 40-300 mm — front medium range
constexpr uint8_t PIN_IR_LONG_LEFT   = A8;   // 2Y0A21, 100-800 mm — left sensor
constexpr uint8_t PIN_IR_MED_RIGHT   = A12;  // 2D120X / 2Y0A41SK, 40-300 mm — right sensor
constexpr uint8_t PIN_IR_LONG_REAR   = A9;   // 2Y0A21, 100-800 mm — rear long range

// Ultrasonic HC-SR04
constexpr uint8_t PIN_US_TRIG = 48;
constexpr uint8_t PIN_US_ECHO = 49;
constexpr unsigned long US_MAX_PULSE_US = 23200;  // ~400 cm timeout

// Battery voltage divider
constexpr uint8_t PIN_BATTERY = A0;

// ── Smoothing ──────────────────────────────────────────────────────
constexpr float IR_EMA_ALPHA = 0.1f;  // EMA low-pass filter coefficient (0-1, lower = smoother)
constexpr float US_EMA_ALPHA = 0.2f;  // EMA for ultrasonic (lower = smoother)
constexpr float US_MAX_JUMP_CM = 5.0f; // reject readings that jump more than this from previous

class percepetion
{
private:
    // ── IMU ────────────────────────────────────────────────────────
    Adafruit_BNO08x   bno08x;
    sh2_SensorValue_t  sensorValue;

    // ── IR sensor EMA state (low-pass filtered ADC values) ──────────
    float irMedFrontFiltered;
    float irLongLeftFiltered;
    float irMedRightFiltered;
    float irLongRearFiltered;

    // ── Ultrasonic ────────────────────────────────────────────────
    float usDistanceCm;
    float usLastValidCm;    // last accepted reading (for spike rejection)
    int   usRejectCount;    // consecutive rejected readings

    // ── Helpers ───────────────────────────────────────────────────
    void  readIR();
    void  readUltrasonic();

    // Convert raw ADC (0-1023) to distance in mm.
    // NOTE: these are starting-point approximations — calibrate with
    //       measured data and replace with your own curve fit.
    float irMedFrontRawToMm(int raw)   const;   // 2D120X / 2Y0A41SK (A10)
    float irLongLeftRawToMm(int raw)   const;   // 2Y0A21 (A8)
    float irMedRightRawToMm(int raw)   const;   // 2D120X / 2Y0A41SK (A12)
    float irLongRearRawToMm(int raw)   const;   // 2Y0A21 (A9)

    float gyro_bias; // for simple gyro zeroing

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
    float getIRMedFront();    // front-facing medium-range
    float getIRLongLeft();    // left-facing  long-range
    float getIRMedRight();    // right-facing medium-range
    float getIRLongRear();    // rear-facing  long-range

    // Raw ADC values (useful for calibration / serial logging)
    int   getIRMedFrontRaw();
    int   getIRLongLeftRaw();
    int   getIRMedRightRaw();
    int   getIRLongRearRaw();

    // ── Ultrasonic getter ─────────────────────────────────────────
    float getUltrasonicCm();  // distance in cm, 0 if no echo

    // ── Battery ───────────────────────────────────────────────────
    int   getBatteryRaw();            // raw ADC 0-1023
    float getBatteryVoltage();        // estimated voltage (V)
    bool  isBatteryLow();             // true when below safe threshold

    void calibrateGyro();              // call when robot is stationary to set gyro zero

    // Returns true if any sensor reads below threshold_mm
    bool isObstacleTooClose(float threshold_mm);

};

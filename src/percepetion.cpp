#include "percepetion.h"

// ═══════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════

percepetion::percepetion()
    : bno08x(-1),
      sensorValue{},
      irMedFrontFiltered(0.0f),
      irLongLeftFiltered(0.0f),
      irMedRightFiltered(0.0f),
      irLongRearFiltered(0.0f),
      usDistanceCm(0.0f),
      usLastValidCm(-1.0f),
      usRejectCount(0)
{}

percepetion::~percepetion() {}

// ═══════════════════════════════════════════════════════════════════
//  Initialisation
// ═══════════════════════════════════════════════════════════════════

bool percepetion::init()
{
    // ── IR sensor pins (analog inputs default, but be explicit) ───
    pinMode(PIN_IR_MED_FRONT,  INPUT);
    pinMode(PIN_IR_LONG_LEFT,  INPUT);
    pinMode(PIN_IR_MED_RIGHT,  INPUT);
    pinMode(PIN_IR_LONG_REAR,  INPUT);

    // ── Ultrasonic ────────────────────────────────────────────────
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);

    // ── Battery ───────────────────────────────────────────────────
    pinMode(PIN_BATTERY, INPUT);

    // ── IMU (BNO08x over I2C) ────────────────────────────────────
    if (!bno08x.begin_I2C()) return false;
    if (!bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000))
        return false;

    // Seed EMA filters with first reading so they start at a real value
    irMedFrontFiltered = (float)analogRead(PIN_IR_MED_FRONT);
    irLongLeftFiltered = (float)analogRead(PIN_IR_LONG_LEFT);
    irMedRightFiltered = (float)analogRead(PIN_IR_MED_RIGHT);
    irLongRearFiltered = (float)analogRead(PIN_IR_LONG_REAR);

    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Main update — call once per loop()
// ═══════════════════════════════════════════════════════════════════

void percepetion::update()
{
    // IMU
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000);
    }
    bno08x.getSensorEvent(&sensorValue);

    // IR sensors (EMA low-pass filter)
    readIR();

    // Ultrasonic
    readUltrasonic();
}

// ═══════════════════════════════════════════════════════════════════
//  Private helpers
// ═══════════════════════════════════════════════════════════════════

void percepetion::readIR()
{
    irMedFrontFiltered += IR_EMA_ALPHA * ((float)analogRead(PIN_IR_MED_FRONT) - irMedFrontFiltered);
    irLongLeftFiltered += IR_EMA_ALPHA * ((float)analogRead(PIN_IR_LONG_LEFT) - irLongLeftFiltered);
    irMedRightFiltered += IR_EMA_ALPHA * ((float)analogRead(PIN_IR_MED_RIGHT) - irMedRightFiltered);
    irLongRearFiltered += IR_EMA_ALPHA * ((float)analogRead(PIN_IR_LONG_REAR) - irLongRearFiltered);
}

void percepetion::readUltrasonic()
{
    // Standard HC-SR04 trigger sequence
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    unsigned long pulse = pulseIn(PIN_US_ECHO, HIGH, US_MAX_PULSE_US);
    float rawCm = (pulse > 0) ? (float)pulse / 58.0f : 0.0f;

    // No echo — keep previous filtered value
    if (rawCm <= 0.0f) return;

    // Spike rejection: if reading jumps too far from last valid, reject it.
    // After 3 consecutive rejects, accept anyway (the robot actually moved).
    if (usLastValidCm > 0.0f &&
        fabsf(rawCm - usLastValidCm) > US_MAX_JUMP_CM &&
        usRejectCount < 3)
    {
        usRejectCount++;
        return;
    }

    usRejectCount = 0;
    usLastValidCm = rawCm;

    // EMA low-pass filter
    if (usDistanceCm <= 0.0f) {
        usDistanceCm = rawCm;  // seed on first valid reading
    } else {
        usDistanceCm += US_EMA_ALPHA * (rawCm - usDistanceCm);
    }
}

// ── IR distance conversion (power-law curve fit) ──────────────────
// Uses D = k * pow(ADC, exponent) as shown in the course tutorial.
// IMPORTANT: You MUST calibrate these against measured distances.
//
// Method: place sensor at known distances (e.g. every 50 mm), record
// raw ADC values, then fit D = k * ADC^exp (e.g. in MATLAB / Excel).

float percepetion::irMedFrontRawToMm(int raw) const
{
    // Sharp 2D120X / 2Y0A41SK on A10 (front-facing, 40-300 mm range)
    // ok breaks at 27 ID: blue tac on bottom
    if (raw < 15) return 300.0f;
    float mm = 56806.0f * pow((float)raw, -1.166f);
    return constrain(mm, 40.0f, 300.0f);
}

float percepetion::irLongLeftRawToMm(int raw) const
{
    // Sharp 2Y0A21 on A8 (left-facing, 100-800 mm range)
    // breaks at 60cm. will need lpf or averaging ID: 39
    if (raw < 20) return 800.0f;
    float mm = 79426.0f * pow((float)raw, -1.078f);
    return constrain(mm, 100.0f, 800.0f);
}

float percepetion::irMedRightRawToMm(int raw) const
{
    // Sharp 2D120X / 2Y0A41SK on A12 (right sensor, 40-300 mm range)
    // roughly 4mm inaccuracy. then jumps to 20mm at 28cm ID: blue tac on top 
    if (raw < 15) return 300.0f;
    float mm = 16827.0f * pow((float)raw, -0.949f);
    return constrain(mm, 40.0f, 300.0f);
}

float percepetion::irLongRearRawToMm(int raw) const
{
    // Sharp 2Y0A21 on A9 (rear-facing, 100-800 mm range)
    // good but breaks at 65 cm ID: 06
    if (raw < 20) return 800.0f;
    float mm = 128820.0f * pow((float)raw, -1.161f);
    return constrain(mm, 100.0f, 800.0f);
}

// ═══════════════════════════════════════════════════════════════════
//  IMU getter
// ═══════════════════════════════════════════════════════════════════

float percepetion::getGyroZ()
{
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
        return sensorValue.un.gyroscope.z;
    }
    return 0.0f;
}

// ═══════════════════════════════════════════════════════════════════
//  IR getters — filtered distance (mm)
// ═══════════════════════════════════════════════════════════════════

float percepetion::getIRMedFront()
{
    return irMedFrontRawToMm((int)irMedFrontFiltered);
}

float percepetion::getIRLongLeft()
{
    return irLongLeftRawToMm((int)irLongLeftFiltered);
}

float percepetion::getIRMedRight()
{
    return irMedRightRawToMm((int)irMedRightFiltered);
}

float percepetion::getIRLongRear()
{
    return irLongRearRawToMm((int)irLongRearFiltered);
}

// ── Raw ADC (for calibration / logging) ───────────────────────────

int percepetion::getIRMedFrontRaw()
{
    return (int)irMedFrontFiltered;
}

int percepetion::getIRLongLeftRaw()
{
    return (int)irLongLeftFiltered;
}

int percepetion::getIRMedRightRaw()
{
    return (int)irMedRightFiltered;
}

int percepetion::getIRLongRearRaw()
{
    return (int)irLongRearFiltered;
}

// ═══════════════════════════════════════════════════════════════════
//  Ultrasonic getter
// ═══════════════════════════════════════════════════════════════════

float percepetion::getUltrasonicCm()
{
    return usDistanceCm;
}

// ═══════════════════════════════════════════════════════════════════
//  Battery monitoring
// ═══════════════════════════════════════════════════════════════════

int percepetion::getBatteryRaw()
{
    return analogRead(PIN_BATTERY);
}

float percepetion::getBatteryVoltage()
{
    // Based on reference code voltage divider mapping:
    // ADC 717 ≈ 3.5 V,  ADC 860 ≈ 4.2 V  (per-cell LiPo)
    int raw = analogRead(PIN_BATTERY);
    return 3.5f + (float)(raw - 717) * 0.7f / 143.0f;
}

bool percepetion::isBatteryLow()
{
    // Below ~3.5 V per cell is the danger zone for LiPo
    return (analogRead(PIN_BATTERY) < 717);
}

bool percepetion::isObstacleTooClose(float threshold_mm)
{
    return getIRMedFront()  < threshold_mm ||
           getIRLongLeft()  < threshold_mm ||
           getIRMedRight()  < threshold_mm ||
           getIRLongRear()  < threshold_mm;
}

void percepetion::calibrateGyro() {
    long sum = 0;
    uint16_t samples = 500;
    for (uint16_t i = 0; i < samples; ++i) {
        bno08x.getSensorEvent(&sensorValue);
        if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
            sum += sensorValue.un.gyroscope.z;
        }
        delay(5);  // sample at ~200 Hz
    }
    gyro_bias = (float)sum / samples;
}
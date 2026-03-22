#include "percepetion.h"

// ═══════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════

percepetion::percepetion()
    : bno08x(-1),
      sensorValue{},
      irBufIdx(0),
      usDistanceCm(0.0f)
{
    memset(irLongFrontBuf, 0, sizeof(irLongFrontBuf));
    memset(irLongLeftBuf,  0, sizeof(irLongLeftBuf));
    memset(irMedRightBuf,  0, sizeof(irMedRightBuf));
    memset(irMedRearBuf,   0, sizeof(irMedRearBuf));
}

percepetion::~percepetion() {}

// ═══════════════════════════════════════════════════════════════════
//  Initialisation
// ═══════════════════════════════════════════════════════════════════

bool percepetion::init()
{
    // ── IR sensor pins (analog inputs default, but be explicit) ───
    pinMode(PIN_IR_LONG_FRONT, INPUT);
    pinMode(PIN_IR_LONG_LEFT,  INPUT);
    pinMode(PIN_IR_MED_RIGHT,  INPUT);
    pinMode(PIN_IR_MED_REAR,   INPUT);

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

    // Pre-fill IR buffers with a first read so averages are valid
    for (uint8_t i = 0; i < IR_FILTER_SAMPLES; ++i) {
        readIR();
        irBufIdx = (irBufIdx + 1) % IR_FILTER_SAMPLES;
    }

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

    // IR sensors (store into rolling buffer)
    readIR();
    irBufIdx = (irBufIdx + 1) % IR_FILTER_SAMPLES;

    // Ultrasonic
    readUltrasonic();
}

// ═══════════════════════════════════════════════════════════════════
//  Private helpers
// ═══════════════════════════════════════════════════════════════════

void percepetion::readIR()
{
    irLongFrontBuf[irBufIdx] = analogRead(PIN_IR_LONG_FRONT);
    irLongLeftBuf [irBufIdx] = analogRead(PIN_IR_LONG_LEFT);
    irMedRightBuf [irBufIdx] = analogRead(PIN_IR_MED_RIGHT);
    irMedRearBuf  [irBufIdx] = analogRead(PIN_IR_MED_REAR);
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
    usDistanceCm = (pulse > 0) ? (float)pulse / 58.0f : 0.0f;
}

int percepetion::averageBuf(const int *buf, uint8_t len) const
{
    long sum = 0;
    for (uint8_t i = 0; i < len; ++i) sum += buf[i];
    return (int)(sum / len);
}

// ── IR distance conversion (power-law curve fit) ──────────────────
// Uses D = k * pow(ADC, exponent) as shown in the course tutorial.
// IMPORTANT: You MUST calibrate these against measured distances.
//
// Method: place sensor at known distances (e.g. every 50 mm), record
// raw ADC values, then fit D = k * ADC^exp (e.g. in MATLAB / Excel).

float percepetion::irLongFrontRawToMm(int raw) const
{
    // Sharp 2Y0A21 on A9 (front-facing, 100-800 mm range)
    // Calibrated curve fit: D_cm = 4577.8 * ADC^-0.939 - 2
    if (raw < 20) return 800.0f;
    float mm = (4577.8f * pow((float)raw, -0.939f) - 2.0f) * 10.0f;
    return constrain(mm, 100.0f, 800.0f);
}

float percepetion::irLongLeftRawToMm(int raw) const
{
    // Sharp 2Y0A21 on A8 (left-facing, 100-800 mm range)
    // Calibrated curve fit: D_cm = 4754.1 * ADC^-0.98
    if (raw < 20) return 800.0f;
    float mm = 4754.1f * pow((float)raw, -0.98f) * 10.0f;
    return constrain(mm, 100.0f, 800.0f);
}

float percepetion::irMedRightRawToMm(int raw) const
{
    // Sharp 2D120X / 2Y0A41SK on A12 (right sensor, 40-300 mm range)
    // Calibrated curve fit: D_cm = 2502.3 * ADC^-1.001
    if (raw < 15) return 300.0f;
    float mm = 2502.3f * pow((float)raw, -1.001f) * 10.0f;
    return constrain(mm, 40.0f, 300.0f);
}

float percepetion::irMedRearRawToMm(int raw) const
{
    // Sharp 2D120X / 2Y0A41SK on A13 (left/rear sensor, 40-300 mm range)
    // Calibrated curve fit: D_cm = 2421.2 * ADC^-0.992
    if (raw < 15) return 300.0f;
    float mm = 2421.2f * pow((float)raw, -0.992f) * 10.0f;
    return constrain(mm, 40.0f, 300.0f);
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

float percepetion::getIRLongFront()
{
    return irLongFrontRawToMm(averageBuf(irLongFrontBuf, IR_FILTER_SAMPLES));
}

float percepetion::getIRLongLeft()
{
    return irLongLeftRawToMm(averageBuf(irLongLeftBuf, IR_FILTER_SAMPLES));
}

float percepetion::getIRMedRight()
{
    return irMedRightRawToMm(averageBuf(irMedRightBuf, IR_FILTER_SAMPLES));
}

float percepetion::getIRMedRear()
{
    return irMedRearRawToMm(averageBuf(irMedRearBuf, IR_FILTER_SAMPLES));
}

// ── Raw ADC (for calibration / logging) ───────────────────────────

int percepetion::getIRLongFrontRaw()
{
    return averageBuf(irLongFrontBuf, IR_FILTER_SAMPLES);
}

int percepetion::getIRLongLeftRaw()
{
    return averageBuf(irLongLeftBuf, IR_FILTER_SAMPLES);
}

int percepetion::getIRMedRightRaw()
{
    return averageBuf(irMedRightBuf, IR_FILTER_SAMPLES);
}

int percepetion::getIRMedRearRaw()
{
    return averageBuf(irMedRearBuf, IR_FILTER_SAMPLES);
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
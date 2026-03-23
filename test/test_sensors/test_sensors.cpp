/**
 * Sensor test — prints all IR + ultrasonic readings as columns on Serial.
 *
 * Upload with:   pio run -e test_sensors -t upload
 * Monitor with:  pio device monitor -b 115200
 */

#include <Arduino.h>

// ── Pin assignments (must match percepetion.h) ──────────────────────
constexpr uint8_t PIN_IR_MED_FRONT  = A10;  // medium range, front
constexpr uint8_t PIN_IR_LONG_LEFT  = A8;   // long range, left
constexpr uint8_t PIN_IR_MED_RIGHT  = A12;  // medium range, right
constexpr uint8_t PIN_IR_LONG_REAR  = A9;   // long range, rear

constexpr uint8_t PIN_US_TRIG = 48;
constexpr uint8_t PIN_US_ECHO = 49;
constexpr unsigned long US_MAX_PULSE_US = 23200;

// ── IR distance conversions (copied from percepetion.cpp) ───────────
float irMedFrontToMm(int raw) {
    if (raw < 15) return 300.0f;
    float mm = 2421.2f * pow((float)raw, -0.992f) * 10.0f;
    return constrain(mm, 40.0f, 300.0f);
}

float irLongLeftToMm(int raw) {
    if (raw < 20) return 800.0f;
    float mm = 4754.1f * pow((float)raw, -0.98f) * 10.0f;
    return constrain(mm, 100.0f, 800.0f);
}

float irMedRightToMm(int raw) {
    if (raw < 15) return 300.0f;
    float mm = 2502.3f * pow((float)raw, -1.001f) * 10.0f;
    return constrain(mm, 40.0f, 300.0f);
}

float irLongRearToMm(int raw) {
    if (raw < 20) return 800.0f;
    float mm = (4577.8f * pow((float)raw, -0.939f) - 2.0f) * 10.0f;
    return constrain(mm, 100.0f, 800.0f);
}

// ── Ultrasonic helper ───────────────────────────────────────────────
float readUltrasonicCm() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    unsigned long pulse = pulseIn(PIN_US_ECHO, HIGH, US_MAX_PULSE_US);
    return (pulse > 0) ? (float)pulse / 58.0f : 0.0f;
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_IR_MED_FRONT, INPUT);
    pinMode(PIN_IR_LONG_LEFT, INPUT);
    pinMode(PIN_IR_MED_RIGHT, INPUT);
    pinMode(PIN_IR_LONG_REAR, INPUT);
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);

    delay(500);

    // Print header
    Serial.println(F("Front_mm(M)\tLeft_mm(L)\tRight_mm(M)\tRear_mm(L)\tUS_cm"));
    Serial.println(F("-----------\t----------\t-----------\t----------\t-----"));
}

void loop() {
    float front = irMedFrontToMm(analogRead(PIN_IR_MED_FRONT)) / 10.0f;  // convert to cm for easier reading
    float left  = irLongLeftToMm(analogRead(PIN_IR_LONG_LEFT)) / 10.0f;
    float right = irMedRightToMm(analogRead(PIN_IR_MED_RIGHT)) / 10.0f;
    float rear  = irLongRearToMm(analogRead(PIN_IR_LONG_REAR)) / 10.0f;
    float usCm  = readUltrasonicCm();

    Serial.print(front, 1);  Serial.print('\t');  Serial.print('\t');
    Serial.print(left, 1);   Serial.print('\t');  Serial.print('\t');
    Serial.print(right, 1);  Serial.print('\t');  Serial.print('\t');
    Serial.print(rear, 1);   Serial.print('\t');  Serial.print('\t');
    Serial.println(usCm, 1);

    delay(200);  // ~5 Hz update rate
}

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
    float mm = 56806.0f * pow((float)raw, -1.166f) / 10.0f; // ok breaks at 27 
    return mm;
}

float irLongLeftToMm(int raw) { // breaks at 60cm. will need lpf or averaging 
    float mm = 79426.0f * pow((float)raw, -1.078f);
    return mm;
}

float irMedRightToMm(int raw) {
    float mm = 16827.0f * pow((float)raw, -0.949f) ;
    return mm;
}

float irLongRearToMm(int raw) {
    // float mm = (22392.0f * pow((float)raw, -1.264f));
    // float mm = 4378.16 / (raw - 36.08);
    // float mm  = 3740.1 / (raw - 65.6);
    float mm = 128820* pow((float)raw, -1.161f)/10.0f; // good but breaks at 65 cm 
    return mm;

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
    float front = irMedFrontToMm(analogRead(PIN_IR_MED_FRONT));  // convert to cm for easier reading
    float left  = irLongLeftToMm(analogRead(PIN_IR_LONG_LEFT)) / 10.0f;
    float right = irMedRightToMm(analogRead(PIN_IR_MED_RIGHT)) / 10.0f;
    float rear  = irLongRearToMm(analogRead(PIN_IR_LONG_REAR));
    // float rear = analogRead(PIN_IR_LONG_REAR);
    float usCm  = readUltrasonicCm();

    Serial.print(front, 1);  Serial.print('\t');  Serial.print('\t');
    Serial.print(left, 1);   Serial.print('\t');  Serial.print('\t');
    Serial.print(right, 1);  Serial.print('\t');  Serial.print('\t');
    Serial.print(rear, 1);   Serial.print('\t');  Serial.print('\t');
    Serial.println(usCm, 1);

    delay(200);  // ~5 Hz update rate
}

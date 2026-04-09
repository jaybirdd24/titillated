/**
 * Sensor test — prints readings for the selected sensor on Serial.
 *
 * !! SET THE SENSOR BELOW BEFORE UPLOADING !!
 *
 * Upload with:   pio run -e test_sensors -t upload
 * Monitor with:  pio device monitor -e test_sensors   (115200 baud)
 */

#include <Arduino.h>

// ── Select sensor (uncomment ONE) ────────────────────────────────────────────
//#define SENSOR_FRONT    // A10  2D120X / 2Y0A41SK   medium range  ~40–300 mm
 #define SENSOR_LEFT  // A8   2Y0A21               long range   ~100–800 mm
// #define SENSOR_RIGHT // A12  2D120X / 2Y0A41SK   medium range  ~40–300 mm
// #define SENSOR_REAR  // A9   2Y0A21               long range   ~100–800 mm
// #define SENSOR_US    // trig=48 echo=49           ultrasonic
// ─────────────────────────────────────────────────────────────────────────────

#if defined(SENSOR_FRONT)
  constexpr uint8_t SENSOR_PIN = A10;
  #define SENSOR_NAME "front (A10, medium)"
  #define SENSOR_IS_IR
  #define IR_CONVERT(raw) (56806.0f * pow((float)(raw), -1.166f) / 10.0f)
  #define UNIT "mm"

#elif defined(SENSOR_LEFT)
  constexpr uint8_t SENSOR_PIN = A8;
  #define SENSOR_NAME "left (A8, long)"
  #define SENSOR_IS_IR
  #define IR_CONVERT(raw) (79426.0f * pow((float)(raw), -1.078f) / 10.0f)
  #define UNIT "mm"

#elif defined(SENSOR_RIGHT)
  constexpr uint8_t SENSOR_PIN = A12;
  #define SENSOR_NAME "right (A12, medium)"
  #define SENSOR_IS_IR
  #define IR_CONVERT(raw) (16827.0f * pow((float)(raw), -0.949f) / 10.0f)
  #define UNIT "mm"

#elif defined(SENSOR_REAR)
  constexpr uint8_t SENSOR_PIN = A9;
  #define SENSOR_NAME "rear (A9, long)"
  #define SENSOR_IS_IR
  #define IR_CONVERT(raw) (128820.0f * pow((float)(raw), -1.161f) / 10.0f)
  #define UNIT "mm"

#elif defined(SENSOR_US)
  constexpr uint8_t PIN_US_TRIG = 48;
  constexpr uint8_t PIN_US_ECHO = 49;
  #define SENSOR_NAME "ultrasonic (trig=48 echo=49)"
  #define SENSOR_IS_US
  #define UNIT "cm"

#else
  #error "No sensor selected — uncomment one SENSOR_x define at the top of test_sensors.cpp"
#endif

#ifdef SENSOR_IS_US
constexpr unsigned long US_MAX_PULSE_US = 23200;

static float readUltrasonic() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    unsigned long pulse = pulseIn(PIN_US_ECHO, HIGH, US_MAX_PULSE_US);
    return (pulse > 0) ? (float)pulse / 58.0f : 0.0f;
}
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

#ifdef SENSOR_IS_IR
    pinMode(SENSOR_PIN, INPUT);
#endif
#ifdef SENSOR_IS_US
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);
#endif

    delay(500);

    Serial.print(F("# Sensor: "));
    Serial.println(F(SENSOR_NAME));
    Serial.println(F("raw_adc\tdist_" UNIT));
    Serial.println(F("-------\t--------"));
}

void loop() {
#ifdef SENSOR_IS_IR
    int   raw  = analogRead(SENSOR_PIN);
    float dist = IR_CONVERT(raw);
    Serial.print(raw);
    Serial.print('\t');
    Serial.println(dist, 1);
#endif
#ifdef SENSOR_IS_US
    float dist = readUltrasonic();
    Serial.println(dist, 1);
#endif

    delay(200);
}

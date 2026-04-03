// ─────────────────────────────────────────────────────────────────────────────
// IR Sensor Calibration Sketch
//
// !! SET THE SENSOR BELOW BEFORE UPLOADING !!
//
// Flash:   pio run -e ir_calibrate -t upload
// Monitor: pio device monitor -e ir_calibrate   (115200 baud)
//
// Commands (Serial Monitor):
//   <number>  — capture & record a data point at that distance (mm)
//   h         — reprint the CSV header
//   c         — snapshot current reading
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

// ── Select sensor (uncomment ONE) ────────────────────────────────────────────
// #define SENSOR_FRONT    // A10  2D120X / 2Y0A41SK   ~40–300 mm
// #define SENSOR_LEFT  // A8   2Y0A21              ~100–800 mm
#define SENSOR_RIGHT // A12  2D120X / 2Y0A41SK   ~40–300 mm
// #define SENSOR_REAR  // A9   2Y0A21              ~100–800 mm
// ─────────────────────────────────────────────────────────────────────────────

#if defined(SENSOR_FRONT)
  constexpr uint8_t  SENSOR_PIN  = A10;
  #define            SENSOR_NAME "front (A10)"
#elif defined(SENSOR_LEFT)
  constexpr uint8_t  SENSOR_PIN  = A8;
  #define            SENSOR_NAME "left (A8)"
#elif defined(SENSOR_RIGHT)
  constexpr uint8_t  SENSOR_PIN  = A12;
  #define            SENSOR_NAME "right (A12)"
#elif defined(SENSOR_REAR)
  constexpr uint8_t  SENSOR_PIN  = A9;
  #define            SENSOR_NAME "rear (A9)"
#else
  #error "No sensor selected — uncomment one SENSOR_x define at the top of ir_calibrate.cpp"
#endif

constexpr uint8_t  CAPTURE_SAMPLES  = 100;
constexpr uint8_t  CAPTURE_DELAY_MS = 5;
constexpr uint16_t STREAM_PERIOD_MS = 100;

static int captureAverage() {
    long sum = 0;
    for (uint8_t i = 0; i < CAPTURE_SAMPLES; i++) {
        sum += analogRead(SENSOR_PIN);
        delay(CAPTURE_DELAY_MS);
    }
    return (int)(sum / CAPTURE_SAMPLES);
}

static void printHeader() {
    Serial.println(F("# ── CSV data ──────────────────────────────────────────────"));
    Serial.print(F("dist_mm,raw_adc ("));
    Serial.print(F(SENSOR_NAME));
    Serial.println(')');
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.print(F("# Sensor: "));
    Serial.println(F(SENSOR_NAME));
    printHeader();
    Serial.println(F("# Type a distance in mm then Enter  |  'h' = header  |  'c' = snapshot"));
}

static String inputBuf;

void loop() {
    // ── Live stream ────────────────────────────────────────────────────────
    static unsigned long lastStream = 0;
    if (millis() - lastStream >= STREAM_PERIOD_MS) {
        lastStream = millis();
        Serial.print(F("# live  raw="));
        Serial.println(analogRead(SENSOR_PIN));
    }

    // ── Serial input ───────────────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            inputBuf.trim();
            if (inputBuf.length() == 0) { inputBuf = ""; continue; }

            if (inputBuf.equalsIgnoreCase("h")) {
                printHeader();

            } else if (inputBuf.equalsIgnoreCase("c")) {
                Serial.print(F("# snap  raw="));
                Serial.println(analogRead(SENSOR_PIN));

            } else {
                int dist = inputBuf.toInt();
                if (dist > 0) {
                    Serial.print(F("# Capturing at "));
                    Serial.print(dist);
                    Serial.println(F(" mm ..."));

                    Serial.print(dist);
                    Serial.print(',');
                    Serial.println(captureAverage());

                    Serial.println(F("# Done — move to next distance."));
                } else {
                    Serial.println(F("# Unknown — type a distance in mm (e.g. 150)"));
                }
            }
            inputBuf = "";
        } else {
            inputBuf += c;
        }
    }
}

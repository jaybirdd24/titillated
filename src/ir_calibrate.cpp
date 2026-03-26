// ─────────────────────────────────────────────────────────────────────────────
// IR Sensor Calibration Sketch
//
// Flash:  pio run -e ir_calibrate -t upload
// Monitor: pio device monitor -e ir_calibrate   (115200 baud)
//
// Workflow:
//   1. Place the robot so the sensor under test faces a wall at a known distance
//   2. Wait for the live readings to stabilise
//   3. Type that distance in mm and press Enter
//   4. The board captures 50 samples, averages them, and prints a CSV row
//   5. Repeat for each distance (e.g. 50, 100, 150, 200, 250, 300 mm …)
//   6. Copy/paste the CSV rows into Python / Excel and fit a polynomial
//
// Commands:
//   <number>  — capture & record a data point at that distance (mm)
//   h         — reprint the CSV header
//   c         — print current live readings once (snapshot)
//
// Sensors:
//   front_med  A10   2D120X / 2Y0A41SK   ~40–300 mm
//   left_long  A8    2Y0A21              ~100–800 mm
//   right_med  A12   2D120X / 2Y0A41SK   ~40–300 mm
//   rear_long  A9    2Y0A21              ~100–800 mm
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

constexpr uint8_t  PIN_FRONT         = A10;
constexpr uint8_t  PIN_LEFT          = A8;
constexpr uint8_t  PIN_RIGHT         = A12;
constexpr uint8_t  PIN_REAR          = A9;

constexpr uint8_t  CAPTURE_SAMPLES   = 20;    // samples to average per data point
constexpr uint8_t  CAPTURE_DELAY_MS  = 5;     // ms between each sample
constexpr uint16_t STREAM_PERIOD_MS  = 100;   // live preview rate (ms)

struct Reads { int front, left, right, rear; };

static Reads readAll() {
    return { analogRead(PIN_FRONT), analogRead(PIN_LEFT),
             analogRead(PIN_RIGHT), analogRead(PIN_REAR) };
}

static Reads captureAverage() {
    long sF = 0, sL = 0, sR = 0, sRear = 0;
    for (uint8_t i = 0; i < CAPTURE_SAMPLES; i++) {
        sF    += analogRead(PIN_FRONT);
        sL    += analogRead(PIN_LEFT);
        sR    += analogRead(PIN_RIGHT);
        sRear += analogRead(PIN_REAR);
        delay(CAPTURE_DELAY_MS);
    }
    return { (int)(sF / CAPTURE_SAMPLES), (int)(sL / CAPTURE_SAMPLES),
             (int)(sR / CAPTURE_SAMPLES), (int)(sRear / CAPTURE_SAMPLES) };
}

static void printHeader() {
    Serial.println(F("# ── CSV data (copy rows without the '#' prefix) ──────────────"));
    Serial.println(F("dist_mm,front_raw(A10),left_raw(A8),right_raw(A12),rear_raw(A9)"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    printHeader();
    Serial.println(F("# Type a distance in mm then Enter  |  'h' = header  |  'c' = snapshot"));
    Serial.println(F("# Live readings every 300 ms:"));
}

static String inputBuf;

void loop() {
    // ── Live stream ────────────────────────────────────────────────────────
    static unsigned long lastStream = 0;
    if (millis() - lastStream >= STREAM_PERIOD_MS) {
        lastStream = millis();
        Reads r = readAll();
        Serial.print(F("# live  front="));  Serial.print(r.front);
        Serial.print(F("  left="));         Serial.print(r.left);
        Serial.print(F("  right="));        Serial.print(r.right);
        Serial.print(F("  rear="));         Serial.println(r.rear);
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
                Reads r = readAll();
                Serial.print(F("# snap  front=")); Serial.print(r.front);
                Serial.print(F("  left="));        Serial.print(r.left);
                Serial.print(F("  right="));       Serial.print(r.right);
                Serial.print(F("  rear="));        Serial.println(r.rear);

            } else {
                int dist = inputBuf.toInt();
                if (dist > 0) {
                    Serial.print(F("# Capturing "));
                    Serial.print(CAPTURE_SAMPLES);
                    Serial.print(F(" samples at "));
                    Serial.print(dist);
                    Serial.println(F(" mm ..."));

                    Reads avg = captureAverage();

                    // CSV row
                    Serial.print(dist);      Serial.print(',');
                    Serial.print(avg.front); Serial.print(',');
                    Serial.print(avg.left);  Serial.print(',');
                    Serial.print(avg.right); Serial.print(',');
                    Serial.println(avg.rear);

                    Serial.println(F("# Done — move to next distance, type it, press Enter."));
                } else {
                    Serial.println(F("# Unknown input — type a distance in mm (e.g. 150)"));
                }
            }
            inputBuf = "";
        } else {
            inputBuf += c;
        }
    }
}

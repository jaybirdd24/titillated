#include <Arduino.h>
#include "percepetion.h"

static percepetion perception;

void setup() {
    Serial.begin(115200);
    perception.init();
    delay(2000);
    Serial.println("US(mm)  IR_RIGHT(mm)  DIFF(us-ir)");
}

void loop() {
    perception.update();

    float us_mm  = perception.getUltrasonicCm() * 10.0f;
    float ir_mm  = perception.getIRMedRight();
    float diff   = us_mm - ir_mm;

    Serial.print(us_mm, 1);
    Serial.print("\t");
    Serial.print(ir_mm, 1);
    Serial.print("\t");
    Serial.println(diff, 1);

    delay(100);
}

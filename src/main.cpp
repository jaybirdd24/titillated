#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine;

void setup()
{
    Serial.begin(9600);
    perception.init();
    motors.enable();

    // Print header for serial monitor / PuTTY
    Serial.println("IR_R_raw,IR_R_mm,IR_LRear_raw,IR_LRear_mm");
}

void loop()
{
    perception.update();
    stateMachine.fsmUpdate();

    // ── IR sensor test output ────────────────────────────────
    // Prints raw ADC and converted mm for both medium IR sensors
    // Format: CSV so you can paste into Excel/MATLAB for calibration
    Serial.print(perception.getIRMedRightRaw());
    Serial.print(",");
    Serial.print(perception.getIRMedRight(), 1);
    Serial.print(",");
    Serial.print(perception.getIRLongRearRaw());
    Serial.print(",");
    Serial.println(perception.getIRLongRear(), 1);

    delay(100);  // 10 Hz — fast enough for calibration, slow enough to read
}

#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"

static const int TEST_SPEED    = 300;   // 0–1000
static const int TEST_DURATION = 8000;  // ms
static const int RAMP_TIME     = 800;   // ms to ramp up / ramp down

static percepetion perception;
static movement    motors(&perception);

void setup()
{
    Serial.begin(115200);
    perception.init();
    motors.enable();

    delay(2000);
    Serial.println("RUNNING: Strafe Right");

    unsigned long start = millis();
    while (millis() - start < TEST_DURATION) {
        unsigned long elapsed = millis() - start;
        unsigned long remaining = TEST_DURATION - elapsed;

        // Ramp up at start, ramp down at end
        int speed;
        if (elapsed < RAMP_TIME) {
            speed = (int)(TEST_SPEED * elapsed / RAMP_TIME);
        } else if (remaining < RAMP_TIME) {
            speed = (int)(TEST_SPEED * remaining / RAMP_TIME);
        } else {
            speed = TEST_SPEED;
        }

        perception.update();
        motors.MoveRight(speed);
        Serial.print("GYRO_Z: ");
        Serial.println(perception.getGyroZ(), 4);
    }
    motors.Stop();

    Serial.println("DONE:    Strafe Right");
}

void loop() {}

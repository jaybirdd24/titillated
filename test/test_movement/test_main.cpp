#include <Arduino.h>
#include <SoftwareSerial.h>
#include "percepetion.h"
#include "movement.h"

static const int  TEST_SPEED    = 300;   // 0–1000
static const int  TEST_DURATION = 2000;  // ms per movement
static const int  TEST_PAUSE    = 500;   // ms between movements

static SoftwareSerial WirelessSerial(10, 11);  // RX, TX  (HC-12 module)

static percepetion perception;
static movement    motors(&perception);

static void runMovement(void (*moveFn)(int), const char *label)
{
    WirelessSerial.print("RUNNING: ");
    WirelessSerial.println(label);

    unsigned long start = millis();
    while (millis() - start < TEST_DURATION) {
        perception.update();
        moveFn(TEST_SPEED);
        WirelessSerial.print("  gyroZ: ");
        WirelessSerial.println(perception.getGyroZ(), 4);
    }
    motors.Stop(true);

    WirelessSerial.print("DONE:    ");
    WirelessSerial.println(label);
    delay(TEST_PAUSE);
}

static void doForward (int s) { motors.MoveForward(s);  }
static void doBackward(int s) { motors.MoveBackward(s); }
static void doLeft    (int s) { motors.MoveLeft(s);     }
static void doRight   (int s) { motors.MoveRight(s);    }

void setup()
{
    WirelessSerial.begin(115200);
    perception.init();
    motors.enable();

    delay(2000);  // time to position robot before tests begin
    WirelessSerial.println("=== Movement Tests Start ===");

    runMovement(doForward,  "Drive Forward");
    runMovement(doBackward, "Drive Backward");
    runMovement(doLeft,     "Strafe Left");
    runMovement(doRight,    "Strafe Right");

    WirelessSerial.println("=== Movement Tests Complete ===");
}

void loop()
{
    perception.update();
    WirelessSerial.print("gyroZ: ");
    WirelessSerial.println(perception.getGyroZ(), 4);
    delay(100);
}

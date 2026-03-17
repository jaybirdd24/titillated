#include <Arduino.h>
#include "percepetion.h"
#include "movement.h"
#include "fsm.h"

percepetion perception;
movement    motors(&perception);
fsm         stateMachine;

void setup()
{
    Serial.begin(115200);
    perception.init();
    motors.enable();
}

void loop()
{
    perception.update();
    stateMachine.fsmUpdate();
}

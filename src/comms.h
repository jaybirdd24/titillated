#pragma once

#include <Arduino.h>

class comms {
public:
    void init(unsigned long baud = 115200);
    void sendCSV(unsigned long timeMs, float x, float y, float yaw);
};

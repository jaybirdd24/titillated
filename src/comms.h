#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

// HC-12 wireless module — defined in main.cpp, pins set there
extern SoftwareSerial WirelessSerial;

class comms
{
    private:

    public:
        comms();
        ~comms();
};
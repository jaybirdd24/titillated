#pragma once

#include <Adafruit_BNO08x.h>

class percepetion
{
    private:
        Adafruit_BNO08x bno08x;
        sh2_SensorValue_t sensorValue;

    public:
        percepetion();
        ~percepetion();

        bool init();           // call once in setup; returns false if IMU not found
        void update();         // call each loop iteration to poll sensor
        float getGyroZ();      // angular velocity around Z axis (rad/s), + = CCW
};

#include "percepetion.h"

percepetion::percepetion() : bno08x(-1)
{
}

percepetion::~percepetion()
{
}

bool percepetion::init()
{
    if (!bno08x.begin_I2C()) return false;
    return bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000);
}

void percepetion::update()
{
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000);
    }
    bno08x.getSensorEvent(&sensorValue);
}

float percepetion::getGyroZ()
{
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
        return sensorValue.un.gyroscope.z;
    }
    return 0.0f;
}

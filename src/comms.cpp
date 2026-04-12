#include "comms.h"

void comms::init(unsigned long baud) {
    Serial1.begin(baud);
    Serial1.println("T_ms,X_mm,Y_mm,Yaw_deg");
}

void comms::sendCSV(unsigned long timeMs, float x, float y, float yaw) {
    Serial1.print(timeMs);
    Serial1.print(',');
    Serial1.print(x, 1);
    Serial1.print(',');
    Serial1.print(y, 1);
    Serial1.print(',');
    Serial1.println(yaw, 1);
}

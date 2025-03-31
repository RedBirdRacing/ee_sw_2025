#pragma once

#include <CAN.h>
#include "config.h"

class CANHandler {
public:
    bool begin();
    bool receiveTorqueCommand(float &torqueCommand);

private:
    float rawToTorque(uint16_t rawValue);
};
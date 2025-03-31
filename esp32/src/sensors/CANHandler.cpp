#include "sensors/CANHandler.h"

bool CANHandler::begin() {
    if (!CAN.begin(500E3)) {
        return false;
    }

    // Set CAN pins
    CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
    return true;
}

bool CANHandler::receiveTorqueCommand(float &torqueCommand) {
    int packetSize = CAN.parsePacket();
    if (packetSize == 2) { 
        uint16_t torqueRaw = (CAN.read() << 8) | CAN.read();
        torqueCommand = rawToTorque(torqueRaw);
        return true;
    }
    return false;
}

float CANHandler::rawToTorque(uint16_t rawValue) {
    return rawValue / 100.0f;  // Example scaling factor
}
#include <can.h>
#include <mcp2515.h>
#include <arduino.h>
#include <SPI.h>

void setup(){

    MCP2515 mcp2515(13);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    struct can_frame tx_msg;
    tx_msg.can_id = 0x169;
    tx_msg.can_dlc = 3;
    tx_msg.data[0] = 0x11;
    tx_msg.data[1] = 0x22;
    tx_msg.data[2] = 0x33;
    mcp2515.sendMessage(&tx_msg);

    uint32_t lastLEDtick = 0;

    while (1){

      mcp2515.sendMessage(&tx_msg);

    } 
}


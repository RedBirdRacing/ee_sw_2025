#include "pinMap.h"
#include "Pedal.h"
#include <mcp2515.h>

// uint8_t pin_in[8] = {A1, A2};
uint8_t pin_out[8] = {BTN1, BTN2, BTN3, BTN4, LED1, LED2, LED3};

MCP2515 mcp2515(10);

Pedal pedal = Pedal(A1, A2, millis());

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;


void setup()
{
    // Init hamdler for pedals is done before setup() (pins init included)

    // Init other pins
    for (int i = 0; i < 7; i++)
        pinMode(pin_out[i], OUTPUT);

    // Init mcp2515 
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // 8MHZ for testing on uno
    mcp2515.setNormalMode();

    // Init serial for testing
    while (!Serial);
      Serial.begin(9600);
}

void loop()
{
    // Update pedal value and send through canbus
    pedal.pedal_update(millis());
    pedal.pedal_can_frame_update(&tx_throttle_msg);
    mcp2515.sendMessage(&tx_throttle_msg);

    // mcp2515.sendMessage(&tx_throttle_msg);
    // uint32_t lastLEDtick = 0;
    // if (mcp2515.readMessage(&rx_msg) == MCP2515::ERROR_OK)
    // {
    //     // Commented out as currenlty no need to include receive functionality
    //     // if (rx_msg.can_id == 0x522)
    //     //     for (int i = 0; i < 8; i++)
    //     //         digitalWrite(pin_out[i], (rx_msg.data[0] >> i) & 0x01);
    // }
}

#include "pinMap.h"
#include <can.h>
#include <mcp2515.h>

uint8_t pin_in[8] = {ADC1, ADC2};
uint8_t pin_out[8] = {BTN1, BTN2, BTN3, BTN4, LED1, LED2, LED3};

can_frame tx_msg;
can_frame rx_msg;

uint16_t get_throttle_torque_val(float throttle_val)
{
    uint16_t throttle_val = analogRead(ADC1);

    // Currently just map a linear relationship, with 5V as 100% torque and 0V as 0% torque

    // Scale up the value for canbus
    // For torque, maximum canbus value is 32760, which is 32 times the maximum value of analogRead (1023)
    throttle_val *= 20;

    return throttle_val;
}

can_frame get_throttle_can_msg()
{
    can_frame tx_msg; // Init can_frame

    // Get the throttle sensor voltage value
    float throttle_val;

    // Set the can_frame according to the torque map
    uint16_t throttle_torque_val = get_throttle_torque_val(throttle_val);

    tx_msg.can_id = 201;
    tx_msg.can_dlc = 3;
    tx_msg.data[0] = 0x90; //0x90 for torque, 0x31 for speed
    tx_msg.data[1] = throttle_torque_val % 0xFF;
    tx_msg.data[2] = (throttle_torque_val >> 8) & 0xFF;

    return tx_msg;
}

void setup()
{
    // Init pins
    for (int i = 0; i <  2; i++)
        pinMode(pin_in[i], INPUT);
    for (int i = 0; i < 7; i++)
        pinMode(pin_out[i], OUTPUT);

    MCP2515 mcp2515(PC0); // CS pin PC0
    mcp2515.reset();
    mcp2515.setBierate(CAN_500KBPS, MCP_20MHZ);
    mcp.setNormalMode();
}

void loop()
{
    tx_msg = get_throttle_can_msg();

    uint32_t lastLEDtick = 0;
    if (mcp2515.readMessage(&rx_msg) == MCP2515::ERROR_OK)
    {
        if (rx_msg.can_id == 0x522)
            for (int i = 0; i < 8; i++)
                digitalWrite(pin_out[i], (rx_msg.data[0] >> i) & 0x01);
    }

    mcp2515.sendMessage(tx_msg);
}

// #include "Pedal.h"
// #include "pinMap.h"

// #include <can.h>
// #include <mcp2515.h>

// Pedal throttle(ADC1, ADC2, millis(), 256);

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   throttle.pedalUpdate(millis());
// }

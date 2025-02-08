// #include <CAN.h>

#include "pinMap.h"
// #include <can.h>
#include <mcp2515.h>

uint8_t pin_in[8] = {ADC1, ADC2};
uint8_t pin_out[8] = {BTN1, BTN2, BTN3, BTN4, LED1, LED2, LED3};

MCP2515 mcp2515(PC0); // CS pin PC0

struct can_frame tx_msg;
struct can_frame rx_msg;

float MAX_THROTTLE_VAL = 4.3;
float MIN_THROTTLE_VAL = 0.7;
float THROTTLE_LOWER_DEADZONE_MAX = 0.5;
float THORTTLE_UPPER_DEADZONE_MIN = 4.5;
/*
Between 0V and THROTTLE_LOWER_DEADZONE_MIN: Error for open circuit
Between THROTTLE_LOWER_DEADZONE_MIN and MIN_THROTTLE_VAL: 0% Torque
Between MIN_THROTTLE_VAL and MAX_THROTTLE_VAL: Linear relationship 
Between MAX_THROTTLE_VAL and THORTTLE_UPPER_DEADZONE_MIN: 100% Torque
Between THORTTLE_UPPER_DEADZONE_MIN and 5V: Error for short circuit
*/

uint16_t get_throttle_torque_val(float throttle_val)
{
    float throttle_volt = analogRead(ADC1) / 1024 * 5; // Outputs a value between 0 and 1023 inclusive

    uint16_t ret = 0;

    if (throttle_volt < THROTTLE_LOWER_DEADZONE_MAX)
        ret = 0;
    else if (throttle_volt < MIN_THROTTLE_VAL) 
        ret = 0;
    else if (throttle_volt < MAX_THROTTLE_VAL)
        // Scale up the value for canbus
        // For torque, maximum canbus value is 32760
        ret = (throttle_volt - MIN_THROTTLE_VAL) / (MAX_THROTTLE_VAL - MIN_THROTTLE_VAL) * 32760;
    else if (throttle_volt < THORTTLE_UPPER_DEADZONE_MIN)
        ret = 32760;
    else
        ret = 0;

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

    // MCP2515 mcp2515(PC0); // CS pin PC0
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();
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

    mcp2515.sendMessage(&tx_msg);
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

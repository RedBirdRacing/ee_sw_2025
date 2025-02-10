// #include <CAN.h>

#include "pinMap.h"
// #include <can.h>
#include <mcp2515.h>

uint8_t pin_in[8] = {ADC1, ADC2};
uint8_t pin_out[8] = {BTN1, BTN2, BTN3, BTN4, LED1, LED2, LED3};

MCP2515 mcp2515(PC0); // CS pin PC0

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

const float MAX_THROTTLE_IN_VOLT = 4.3;
const float MIN_THROTTLE_IN_VOLT = 0.7;
const float THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT = 0.5;
const float THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT = 4.5;

const int MAX_THROTTLE_OUT_VAL = 32760; // Maximum torque value is 32760 for mcp2515
const int MIN_THROTTLE_OUT_VAL = 300; // Minium torque value tested is 300 (TBC)

uint16_t get_throttle_torque_val()
{
    float throttle_volt = analogRead(ADC1) / 1024 * 5; // Converts analogRead output to a float between 0V and 5V

    uint16_t ret = 0;

    /*
    Between 0V and THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT: Error for open circuit
    Between THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT and MIN_THROTTLE_IN_VOLT: 0% Torque
    Between MIN_THROTTLE_IN_VOLT and MAX_THROTTLE_IN_VOLT: Linear relationship 
    Between MAX_THROTTLE_IN_VOLT and THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT: 100% Torque
    Between THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT and 5V: Error for short circuit
    */
    if (throttle_volt < THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT)
        ret = 0;
    else if (throttle_volt < MIN_THROTTLE_IN_VOLT) 
        ret = MIN_THROTTLE_OUT_VAL;
    else if (throttle_volt < MAX_THROTTLE_IN_VOLT)
        // Scale up the value for canbus
        ret = (throttle_volt - MIN_THROTTLE_IN_VOLT) / (MAX_THROTTLE_IN_VOLT - MIN_THROTTLE_IN_VOLT) * MAX_THROTTLE_OUT_VAL;
    else if (throttle_volt < THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT)
        ret = MAX_THROTTLE_OUT_VAL;
    else
        ret = 0;

    return ret;
}

uint16_t update_can_tx_throttle_msg()
{
    // Set the can_frame according to the torque map
    uint16_t throttle_torque_val = get_throttle_torque_val();

    if (throttle_torque_val == 0) // Error in reading torque value
      return 1;

    tx_throttle_msg.can_id = 201;
    tx_throttle_msg.can_dlc = 3;
    tx_throttle_msg.data[0] = 0x90; //0x90 for torque, 0x31 for speed
    tx_throttle_msg.data[1] = throttle_torque_val % 0xFF;
    tx_throttle_msg.data[2] = (throttle_torque_val >> 8) & 0xFF;

    return 0;
}

void setup()
{
    // Init pins
    for (int i = 0; i <  2; i++)
        pinMode(pin_in[i], INPUT);
    for (int i = 0; i < 7; i++)
        pinMode(pin_out[i], OUTPUT);

    // Init mcp2515 
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();
}

void loop()
{
    // Send throttle torque msg if no error
    if (update_can_tx_throttle_msg() == 0)
        mcp2515.sendMessage(&tx_throttle_msg);

    uint32_t lastLEDtick = 0;
    if (mcp2515.readMessage(&rx_msg) == MCP2515::ERROR_OK)
    {
        // Commented out as currenlty no need to include receive functionality
        // if (rx_msg.can_id == 0x522)
        //     for (int i = 0; i < 8; i++)
        //         digitalWrite(pin_out[i], (rx_msg.data[0] >> i) & 0x01);
    }
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

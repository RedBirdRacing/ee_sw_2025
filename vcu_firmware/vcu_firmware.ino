#include "pinMap.h"
#include "Pedal.h"
#include <mcp2515.h>
#include "Debug.h"

// uint8_t pin_in[8] = {A1, A2};

// === Pin setup ===
uint8_t pin_out[4] = {LED1, LED2, LED3, BREAK_OUT};
uint8_t pin_in[4] = {BTN1, BTN2, BTN3, BTN4};

// === CAN + Pedal ===
MCP2515 mcp2515(CS_CAN);
Pedal pedal = Pedal(APPS_5V, APPS_3V3, millis());

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

const int THROTTLE_UPDATE_PERIOD_MILLIS = 50; // Period of sending canbus signal
unsigned long final_throttle_time_millis = 0;  // The last time sent a canbus message

// === Car Status State Machine ===
int car_status = 0;
unsigned long car_status_millis_counter = 0; // Millis counter for 1st and 2nd transitionin states
const int STATUS_1_TIME_MILLIS = 2000; // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
const int BUSSIN_TIME_MILLIS = 2000; // The amount of time that the buzzer will buzz for

/*
Meaning of different car statuses
0:  Just started the car -- Motor should not move, regardless of driver pedal input
1:  1st Transition state -- Driver holds the "Start" button and is on full brakes, lasts for STATUS_1_TIME_MILLIS milliseconds
2:  2nd Transition state -- Buzzer bussin, driver can release "Start" button and brakes
3:  Ready to drive -- Motor starts responding according to the driver pedal input. "Drive mode" LED lights up, indicating driver can press the throttle

Separately, the following will be done outside the status checking part:
1.  Before the "Drive mode" LED lights up, if the throttle pedal is pressed (Throttle input is not euqal to 0), the car_status will return to 0
2.  Before the "Drive mode" LED lights up, the canbus will keep sending "0 torque" messages to the motor
Status Legend:
0: Idle
1: Holding start + brake
2: Buzzer on
3: Drive mode enabled
*/

void setup()
{
    // Init hamdler for pedals is done before setup() (pins init included)

    // Init input pins
    for (int i = 0; i < 4; i++)
        pinMode(pin_in[i], INPUT);
    // Init output pins
    for (int i = 0; i < 4; i++)
        pinMode(pin_out[i], OUTPUT);

    // Init mcp2515 
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // 8MHZ for testing on uno
    mcp2515.setNormalMode();

    // Init serial for testing if DEBUG flag is set to true
    if (DEBUG == true) {
        while (!Serial);
        Serial.begin(9600);
    }

    DBGLN_STATUS("Entered State 0 (Idle)");
}

void loop()
{
    // Update pedal value
    pedal.pedal_update(millis());

    /*
    For the time being:
    BTN1 = "Start" button
    BTN2 = Brake pedal
    LED1 = Buzzer output
    LED2 = "Drive" mode indicator
    */
    DBG_PEDAL("Pedal Value: ");
    DBGLN_PEDAL(pedal.final_pedal_value);

    if (car_status == 0)
    {
        // car_status = 3; // For testing drive mode

        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515.sendMessage(&tx_throttle_msg);
        DBGLN_CAN("Holding 0 torque during state 0");

        if (digitalRead(BTN1) == HIGH && digitalRead(BTN2) == HIGH) // Check if "Start" button and brake is fully pressed
        {
            car_status = 1;
            car_status_millis_counter = millis();
            DBGLN_STATUS("Entered State 1");
        }
    }
    else if (car_status == 1)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515.sendMessage(&tx_throttle_msg);
        DBGLN_CAN("Holding 0 torque during state 1");

        if (digitalRead(BTN1) == LOW || digitalRead(BTN2) == LOW) // Check if "Start" button or brake is not fully pressed
        {
            car_status = 0;
            car_status_millis_counter = millis();
            DBGLN_STATUS("Entered State 0 (Idle)");
        }
        else if (millis() - car_status_millis_counter >= STATUS_1_TIME_MILLIS) // Check if button held long enough
        {
            car_status = 2;
            digitalWrite(LED1, HIGH); // Turn on buzzer
            car_status_millis_counter = millis();
            DBGLN_STATUS("Transition to State 2: Buzzer ON");
        }
    }
    else if (car_status == 2)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515.sendMessage(&tx_throttle_msg);
        DBGLN_CAN("Holding 0 torque during state 2");

        if (millis() - car_status_millis_counter >= BUSSIN_TIME_MILLIS)
        {
            digitalWrite(LED2, HIGH); // Turn on "Drive" mode indicator
            digitalWrite(LED1, LOW); // Turn off buzzer
            car_status = 3;
            DBGLN_STATUS("Transition to State 3: Drive mode");
        }
    }
    else if (car_status == 3)
    {
        // In "Drive mode", car_status won't change, the drvier either continue to drive, or shut off the car
        DBGLN_STATUS("In Drive Mode");
    }
    else
    {
        // Error, idk wtf to do here
        DBGLN_STATUS("ERROR: Invalid car_status encountered!");
    }

    // Pedal update
    if (car_status == 3)
    {   
        // Send pedal value through canbus
        pedal.pedal_can_frame_update(&tx_throttle_msg);
        // The following if block is needed only if we limit the lower bound for canbus cycle period
        // if (millis() - final_throttle_time_millis >= THROTTLE_UPDATE_PERIOD_MILLIS)
        // {
        //     mcp2515.sendMessage(&tx_throttle_msg);
        //     final_throttle_time_millis = millis();
        // }
        mcp2515.sendMessage(&tx_throttle_msg);
        DBGLN_CAN("Throttle CAN frame sent");
    }
    else
    {
        if (pedal.final_pedal_value > MIN_THROTTLE_OUT_VAL)
        {
            car_status = 0;
            car_status_millis_counter = millis(); // Set to current time, in case any counter relies on this
            pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
            mcp2515.sendMessage(&tx_throttle_msg);
            DBGLN_STATUS("Throttle pressed too early — Resetting to State 0");
        }
    }


    // mcp2515.sendMessage(&tx_throttle_msg);
    // uint32_t lastLEDtick = 0;
    // Optional RX handling (disabled for now)
    // if (mcp2515.readMessage(&rx_msg) == MCP2515::ERROR_OK)
    // {
    //     // Commented out as currenlty no need to include receive functionality
    //     // if (rx_msg.can_id == 0x522)
    //     //     for (int i = 0; i < 8; i++)
    //     //         digitalWrite(pin_out[i], (rx_msg.data[0] >> i) & 0x01);
    // }
}

#include "pinMap.h"
#include "Pedal.h"
#include <mcp2515.h>

// uint8_t pin_in[8] = {A1, A2};
uint8_t pin_out[3] = {LED1, LED2, LED3};
uint8_t pin_in[4] = {BTN1, BTN2, BTN3, BTN4};

MCP2515 mcp2515(10);

Pedal pedal = Pedal(A1, A2, millis());

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

int car_status = 0;
int car_status_millis_counter = 0; // Millis counter for 1st and 2nd transitionin states
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
*/


void setup()
{
    // Init hamdler for pedals is done before setup() (pins init included)

    // Init input pins
    for (int i = 0; i < 4; i++)
        pinMode(pin_in[i], INPUT);
    // Init output pins
    for (int i = 0; i < 3; i++)
        pinMode(pin_out[i], OUTPUT);

    // Init mcp2515 
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // 8MHZ for testing on uno
    mcp2515.setNormalMode();

    // // Init serial for testing
    // while (!Serial);
    //   Serial.begin(9600);
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
    // Serial.print(pedal.final_pedal_value);
    // Serial.print(" ");
    // Serial.println(car_status);
    if (car_status == 0)
    {
        if (digitalRead(BTN1) == HIGH && digitalRead(BTN2) == HIGH) // Check if "Start" button and brake is fully pressed
        {
            car_status = 1;
            car_status_millis_counter = millis();
        }
    }
    else if (car_status == 1)
    {
        if (digitalRead(BTN1) == LOW || digitalRead(BTN2) == LOW) // Check if "Start" button or brake is not fully pressed
        {
            car_status = 0;
            car_status_millis_counter = millis();
        }
        else if (millis() - car_status_millis_counter >= STATUS_1_TIME_MILLIS) // Check if button held long enough
        {
            car_status = 2;
            digitalWrite(LED1, HIGH); // Turn on buzzer
            car_status_millis_counter = millis();
        }
    }
    else if (car_status == 2)
    {
        if (millis() - car_status_millis_counter >= BUSSIN_TIME_MILLIS)
        {
            digitalWrite(LED2, HIGH); // Turn on "Drive" mode indicator
            digitalWrite(LED1, LOW); // Turn off buzzer
            car_status = 3;
        }
    }
    else if (car_status == 3)
    {
        // In "Drive mode", car_status won't change, the drvier either continue to drive, or shut off the car
    }
    else
    {
        // Error, idk wtf to do here
    }

    // Pedal update
    if (car_status == 3)
    {   
        // Send pedal value through canbus
        pedal.pedal_can_frame_update(&tx_throttle_msg);
        mcp2515.sendMessage(&tx_throttle_msg);
        // Serial.print(tx_throttle_msg.data[0],tx_throttle_msg.data[2],tx_throttle_msg.data[2]);
        // Serial.print(tx_throttle_msg.can_id);
    }
    else
    {
        if (pedal.final_pedal_value > 325)
        {
            car_status = 0;
            car_status_millis_counter = millis(); // Set to current time, in case any counter relies on this
            pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
            mcp2515.sendMessage(&tx_throttle_msg);
        }
    }


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

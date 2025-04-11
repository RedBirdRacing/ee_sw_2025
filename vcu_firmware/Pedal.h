#ifndef PEDAL_H
#define PEDAL_H

#include "Queue.cpp"
#include "mcp2515.h"

// Constants
const float MAX_THROTTLE_IN_VOLT = 4.8;
const float MIN_THROTTLE_IN_VOLT = 1.7;
const float THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT = 1.6;
const float THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT = 5.1;

const int MAX_THROTTLE_OUT_VAL = 32760; // Maximum torque value is 32760 for mcp2515
const int MIN_THROTTLE_OUT_VAL = 300; // Minium torque value tested is 300 (TBC)

const bool FLIP_MOTOR_OUTPUT_DIRECTION = true; // Flips the direction of motor output

#define ADC_BUFFER_SIZE 128

// Class for generic pedal object
// For Gen 5 car, only throttle pedal is wired through the VCU, so we use Pedal class for Throttle pedal only.
class Pedal {
    public:
        // Two input pins for reading both pedal potentiometer
        // Conversion rate in Hz
        Pedal(int input_pin_1, int input_pin_2, unsigned long millis, int conversion_rate = 1000);   

        // Update function. To be called on every loop and pass the current time in millis
        void pedal_update(int millis);

        // Updates the can_frame with the most update pedal value. To be called on every loop and pass the can_frame by reference.
        void pedal_can_frame_update(can_frame *tx_throttle_msg);

        // Updates the can_frame to send a "0 Torque" value through canbus.
        void pedal_can_frame_stop_motor(can_frame *tx_throttle_msg);

        // Pedal value after filtering and processing
        // Under normal circumstance, should store a value between 0 and 1023 inclusive (translates to 0v - 5v)
        int final_pedal_value;

    private:
        int input_pin_1, input_pin_2;
        int conversion_rate;
        
        // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
        // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
        bool fault;
        int fault_start_millis;

        // Period in millisecond
        int conversion_period;

        // Will rollover every 49 days
        unsigned long previous_millis;

        // Returns true if pedal is faulty
        bool check_pedal_fault(int pedal_1, int pedal_2);

        Queue<int, ADC_BUFFER_SIZE> pedalValue_1;
        Queue<int, ADC_BUFFER_SIZE> pedalValue_2;
};

#endif // PEDAL_H
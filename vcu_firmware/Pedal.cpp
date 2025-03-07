#include "Pedal.h"
#include "Arduino.h"
#include "Signal_Processing.cpp"


// Sinc function of size 128
float SINC_128[128] = {0.017232, 0.002666, -0.013033, -0.026004, -0.032934, -0.031899, -0.022884, -0.007851, 0.009675, 0.025427, 
                        0.035421, 0.036957, 0.029329, 0.014081, -0.005294, -0.024137, -0.037732, -0.042472, -0.036792, -0.021652, 
                        -0.000402, 0.021937, 0.039841, 0.048626, 0.045647, 0.031053, 0.007888, -0.018512, -0.041722, -0.055750, 
                        -0.056553, -0.043139, -0.017994, 0.013320, 0.043353, 0.064476, 0.070758, 0.059540, 0.032321, -0.005306, 
                        -0.044714, -0.076126, -0.090908, -0.083781, -0.054402, -0.007911, 0.045791, 0.093940, 0.123670, 0.125067, 
                        0.093855, 0.033095, -0.046569, -0.128280, -0.191785, -0.217229, -0.189201, -0.100224, 0.047040, 0.239389, 
                        0.454649, 0.664997, 0.841471, 0.958851, 1, 0.958851, 0.841471, 0.664997, 0.454649, 0.239389, 0.047040, 
                        -0.100224, -0.189201, -0.217229, -0.191785, -0.128280, -0.046569, 0.033095, 0.093855, 0.125067, 0.123670, 
                        0.093940, 0.045791, -0.007911, -0.054402, -0.083781, -0.090908, -0.076126, -0.044714, -0.005306, 0.032321, 
                        0.059540, 0.070758, 0.064476, 0.043353, 0.013320, -0.017994, -0.043139, -0.056553, -0.055750, -0.041722, 
                        -0.018512, 0.007888, 0.031053, 0.045647, 0.048626, 0.039841, 0.021937, -0.000402, -0.021652, -0.036792, 
                        -0.042472, -0.037732, -0.024137, -0.005294, 0.014081, 0.029329, 0.036957, 0.035421, 0.025427, 0.009675, 
                        -0.007851, -0.022884, -0.031899, -0.032934, -0.026004, -0.013033};


												

Pedal::Pedal(int input_pin_1, int input_pin_2, unsigned long millis, int conversion_rate = 1000)
            : input_pin_1(input_pin_1), input_pin_2(input_pin_2), previous_millis(millis), conversion_rate(conversion_rate), fault(false)
            {
    // Init pins
    pinMode(input_pin_1, INPUT);
    pinMode(input_pin_2, INPUT);
    conversion_period = 1000 / conversion_rate;

    // Init ADC butters
    for (int i = 0; i < ADC_BUFFER_SIZE; ++i) {
        pedalValue_1.buffer[i] = 0;
        pedalValue_2.buffer[i] = 0;
    }
}

void Pedal::pedal_update(int millis) {
    // If is time to update
    if (millis - previous_millis > conversion_period) {
        // Updating the previous millis 
        previous_millis = millis;
        // Record readings in buffer
        pedalValue_1.push(analogRead(input_pin_1));
        pedalValue_2.push(analogRead(input_pin_2));

        int pedal_filtered_1 = FIR_filter<int>(pedalValue_1.buffer, SINC_128, ADC_BUFFER_SIZE, 6.176445);
        int pedal_filtered_2 = FIR_filter<int>(pedalValue_2.buffer, SINC_128, ADC_BUFFER_SIZE, 6.176445);
        final_pedal_value = average(pedal_filtered_1, pedal_filtered_2);

        // if (Serial) {
        //     Serial.print("Filtered output: ");
        //     Serial.println(average(pedal_filtered_1, pedal_filtered_2));
        // }

        if (check_pedal_fault())
        {
            if (fault) { // Previous scan is already faulty
                if (fault_start_millis - millis > 100) { // Faulty for more than 100 ms 
                    // TODO: Add code for alerting the faulty pedal, and whatever else mandated in rules Ch.2 Section 12.8, 12.9

                    // Turning off the motor is achieved using another digital pin, not via canbus
                    return;
                }
            }
            else {
                fault_start_millis = millis;
            }

            fault = true;
            return;
        }
    }
}

void Pedal::pedal_can_frame_update(can_frame *tx_throttle_msg) {
    float throttle_volt = (float)final_pedal_value * 5 / 1024; // Converts most update pedal value to a float between 0V and 5V

    uint16_t throttle_torque_val = 0;
    /*
    Between 0V and THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT: Error for open circuit
    Between THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT and MIN_THROTTLE_IN_VOLT: 0% Torque
    Between MIN_THROTTLE_IN_VOLT and MAX_THROTTLE_IN_VOLT: Linear relationship 
    Between MAX_THROTTLE_IN_VOLT and THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT: 100% Torque
    Between THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT and 5V: Error for short circuit
    */
    if (throttle_volt < THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT)
        throttle_torque_val = 0;

    else if (throttle_volt < MIN_THROTTLE_IN_VOLT) 
        throttle_torque_val = MIN_THROTTLE_OUT_VAL;

    else if (throttle_volt < MAX_THROTTLE_IN_VOLT)
        // Scale up the value for canbus
        throttle_torque_val = (throttle_volt - MIN_THROTTLE_IN_VOLT) * MAX_THROTTLE_OUT_VAL / (MAX_THROTTLE_IN_VOLT - MIN_THROTTLE_IN_VOLT);

    else if (throttle_volt < THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT)
        throttle_torque_val = MAX_THROTTLE_OUT_VAL;

    else
        throttle_torque_val = 0;

    // if (Serial)
    //     Serial.println(String(throttle_torque_val) + " "+ String(analogRead(A1))+ " "+ String(throttle_volt));

    tx_throttle_msg->can_id = 201;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; //0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = throttle_torque_val & 0xFF;
    tx_throttle_msg->data[2] = (throttle_torque_val >> 8) & 0xFF;

}

bool Pedal::check_pedal_fault() { 
    // TODO: Add logic for determining if pedals are faulty or not

    // Currently the only indication for faulty pedal is just 2 pedal values are more than 10% different

    return true;
}
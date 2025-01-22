#ifndef PEDAL_H
#define PEDAL_H

#include "Queue.cpp"


#define ADC_BUFFER_SIZE 128

// Class for generic pedal object
class Pedal {
    public:
        // Two input pins for reading both pedal potentiometer
        // Conversion rate in Hz
        Pedal(int input_pin_1, int input_pin_2, unsigned long millis, int conversion_rate = 1000);   

        // Update function. To be called on every loop and pass the current time in millis
        void pedalUpdate(int millis);

        // Pedal value after filtering and processing
        int finalPedalValue;

    private:
        int input_pin_1, input_pin_2;
        int conversion_rate;
        
        // If the two potentiometer inputs are too different, the inputs are faulty
        bool fault;

        // Period in millisecond
        int conversion_period;

        // Will rollover every 49 days
        unsigned long previous_millis;

        Queue<int, ADC_BUFFER_SIZE> pedalValue_1;
        Queue<int, ADC_BUFFER_SIZE> pedalValue_2;
};

#endif // PEDAL_H
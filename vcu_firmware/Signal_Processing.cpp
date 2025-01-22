#include "Signal_Processing.h"

// Apply a FIR filter on the signal buffer
// The buffer size must be the same as the kernel
// Filtered output will be stored in the output_buf
template <typename T>
T FIR_filter(T* buffer, float* kernel, int buf_size, float kernel_sum) {
    float sum = 0;

    for (int i = 0; i < buf_size; ++i) {
        sum += buffer[i] * kernel[i];
    }

    // Kernel sum is the sum of all values in the kernel. This normalize the output value
    return sum / kernel_sum;
} 

template <typename T>
T average(T val1, T val2) {
    return (val1 + val2) / 2;
}

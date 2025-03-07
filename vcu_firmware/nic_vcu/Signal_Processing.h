// A library containing simple DSP functions, for ADC filtering, buffer comparisons and more
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H


template <typename T>
T FIR_filter(T* buffer, float* kernel, int buf_size, float kernel_sum);

template <typename T>
T average(T val1, T val2);

#endif
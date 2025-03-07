#include "Queue.h"

template <typename T, int size>
Queue<T, size>::Queue() : queueFull(false), queueEmpty(true), queueCount(0) {}


template <typename T, int size>
void Queue<T, size>::push(T val) {
    for (int i = size - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = val;

    if (!queueFull)
        ++queueCount;
      
    queueFull = (queueCount == size);
}


template <typename T, int size>
T Queue<T, size>::pop() {
    if (queueCount == 0)        // If the queue is empty and attempts to pop an object, the program will end
        exit(0);

    --queueCount;
    queueEmpty = (queueCount == 0);

    return buffer[queueCount];
}


template <typename T, int size>
T Queue<T, size>::getHead() {
    return buffer[queueCount - 1];
}


template <typename T, int size>
bool Queue<T, size>::isEmpty() {
    return queueEmpty;
}


template <typename T, int size>
bool Queue<T, size>::isFull() {
    return queueFull;
}
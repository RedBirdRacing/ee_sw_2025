#ifndef QUEUE_H
#define QUEUE_H

// A simple FIFO object
// This object is completely static
template <typename T, int size>
class Queue {
    public:
        Queue();

        void push(T val);
        T pop();
        T getHead();

        bool isEmpty();
        bool isFull();

        T buffer[size];
        
    private:
        bool queueFull, queueEmpty;
        int queueCount;
};

template <typename T, int size>
class RingBuffer {
public:
    RingBuffer() : head(0), count(0) {}

    void push(T val) {
        buffer[head] = val;
        head = (head + 1) % size;
        if (count < size) ++count;
    }

    void getLinearBuffer(T* out) {
        for (int i = 0; i < count; ++i) {
            out[i] = buffer[(head + i) % size];
        }
    }

    T buffer[size];
    int head;
    int count;
};

#endif // QUEUE_H
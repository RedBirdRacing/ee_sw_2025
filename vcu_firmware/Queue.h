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

#endif // QUEUE_H
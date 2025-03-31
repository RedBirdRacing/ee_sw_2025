#include "sensors/Potentiometer.h"

Potentiometer::Potentiometer(uint8_t pin) : pin(pin) {}

void Potentiometer::begin() {
    pinMode(pin, INPUT);
}

int Potentiometer::readValue() {
    return analogRead(pin);
}
#pragma once

#include <Arduino.h>
#include "config.h"

class Potentiometer {
public:
    Potentiometer(uint8_t pin);
    void begin();
    int readValue();

private:
    uint8_t pin;
};

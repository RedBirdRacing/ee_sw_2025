#include "Pedal.h"
#include "pinMap.h"

Pedal throttle(ADC1, ADC2, millis(), 256);

void setup() {
  Serial.begin(115200);
}

void loop() {
  throttle.pedalUpdate(millis());
}

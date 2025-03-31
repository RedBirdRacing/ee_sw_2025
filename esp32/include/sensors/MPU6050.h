#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

class MPU6050Sensor {
public:
    bool begin();
    void getAccelerometerData(float &x, float &y, float &z);
    void getGyroscopeData(float &x, float &y, float &z);

private:
    Adafruit_MPU6050 mpu;
};
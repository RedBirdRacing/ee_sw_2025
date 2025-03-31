#include "sensors/MPU6050.h"

bool MPU6050Sensor::begin() {
    if (!mpu.begin(MPU6050_ADDRESS)) {
        return false;
    }

    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;
}

void MPU6050Sensor::getAccelerometerData(float &x, float &y, float &z) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;
}

void MPU6050Sensor::getGyroscopeData(float &x, float &y, float &z) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    x = g.gyro.x;
    y = g.gyro.y;
    z = g.gyro.z;
}
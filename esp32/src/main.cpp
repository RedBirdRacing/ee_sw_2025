#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <CAN.h>
#include "config.h"
#include "sensors/Potentiometer.h"
#include "sensors/MPU6050.h"
#include "sensors/CANHandler.h"

// Instances
Potentiometer potentiometer(POTENTIOMETER_PIN);
MPU6050Sensor mpuSensor;
CANHandler canHandler;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Initialize Potentiometer
    potentiometer.begin();
    //Adafruit_MPU6050 mpu;

    //MPU6050Sensor mpuSensor;
    // Initialize MPU6050
    /*if (!mpu.begin()) {
        Serial.println("MPU6050 not detected.");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully.");*/

    // Initialize CAN Bus
    if (!canHandler.begin()) {
        Serial.println("CAN bus initialization failed.");
        while (1);
    }
    Serial.println("CAN bus initialized successfully.");
}

void loop() {
    // Read Potentiometer
    int potValue = potentiometer.readValue();
    Serial.print("Potentiometer Value: ");
    Serial.println(potValue);

    // Read Accelerometer Data
    /*float ax, ay, az;
    mpuSensor.getAccelerometerData(ax, ay, az);
    Serial.print("Accelerometer (X, Y, Z): ");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.println(az);

    // Read Gyroscope Data
    float gx, gy, gz;
    mpuSensor.getGyroscopeData(gx, gy, gz);
    Serial.print("Gyroscope (X, Y, Z): ");
    Serial.print(gx);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.println(gz);*/

    // Read CAN bus messages
    float torqueCommand;
    if (canHandler.receiveTorqueCommand(torqueCommand)) {
        Serial.print("Received Torque Command: ");
        Serial.println(torqueCommand);
    } else {
        Serial.println("No CAN packets received.");
    }

    // Delay for stability
    delay(100);
}
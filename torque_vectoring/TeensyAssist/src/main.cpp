// Teensy 4.1 Motor Control over CAN for Formula Student Car
// Features: Torque Vectoring, Traction Control, Launch Control, ABS (motor-only)

#include <FlexCAN_T4.h>
#include <ADC.h>

// === CAN Bus ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// === Motor CAN IDs ===
// Assume front is weaker than rear
const int motorFL_ID = 0x01; // Front-left motor
const int motorFR_ID = 0x02; // Front-right motor
const int motorRL_ID = 0x03; // Rear-left motor
const int motorRR_ID = 0x04; // Rear-right motor

// === Constants ===
const float maxFrontTorque = 0.5;       // Max torque as percent for front motors
const float maxRearTorque = 1.0;        // Max torque as percent for rear motors
const float maxSlipRatio = 0.15;        // Max allowed slip ratio for traction control
const float launchSpeedThreshold = 2.0; // Speed below which launch control activates (m/s)

// === Placeholder Inputs ===
float wheelSpeedFL = 0.0;
float wheelSpeedFR = 0.0;
float wheelSpeedRL = 0.0;
float wheelSpeedRR = 0.0;
float vehicleSpeed = 0.0; // Could be average of rear wheels
bool brakePressed = false;
bool launchRequested = false;

// === Function to send CAN message ===
void sendMotorCommand(int motorID, float torquePercent)
{
  CAN_message_t msg;
  msg.id = motorID;
  msg.len = 2;
  msg.buf[0] = constrain(torquePercent * 255.0, 0, 255);
  msg.buf[1] = 0;
  can1.write(msg);
}

// === Slip Calculation ===
float computeSlip(float drivenSpeed, float actualSpeed)
{
  if (actualSpeed == 0)
    return 0.0;
  return (drivenSpeed - actualSpeed) / actualSpeed;
}

void setup()
{
  Serial.begin(115200);
  can1.begin();
  can1.setBaudRate(500000);
  // Initialize CAN at 500 kbps
}

void loop()
{
  // === Placeholder: Simulated Inputs (replace with sensor reads) ===
  vehicleSpeed = (wheelSpeedRL + wheelSpeedRR) / 2.0;

  float torqueFL = maxFrontTorque;
  float torqueFR = maxFrontTorque;
  float torqueRL = maxRearTorque;
  float torqueRR = maxRearTorque;

  // === Traction Control ===
  float slipFL = computeSlip(vehicleSpeed, wheelSpeedFL);
  float slipFR = computeSlip(vehicleSpeed, wheelSpeedFR);
  float slipRL = computeSlip(vehicleSpeed, wheelSpeedRL);
  float slipRR = computeSlip(vehicleSpeed, wheelSpeedRR);

  if (slipFL > maxSlipRatio)
    torqueFL *= 0.5;
  if (slipFR > maxSlipRatio)
    torqueFR *= 0.5;
  if (slipRL > maxSlipRatio)
    torqueRL *= 0.5;
  if (slipRR > maxSlipRatio)
    torqueRR *= 0.5;

  // === Launch Control ===
  if (launchRequested && vehicleSpeed < launchSpeedThreshold)
  {
    torqueRL = maxRearTorque;
    torqueRR = maxRearTorque;
    torqueFL = maxFrontTorque;
    torqueFR = maxFrontTorque;
  }

  // === ABS-like Regen Limiting ===
  if (brakePressed)
  {
    if (wheelSpeedFL < vehicleSpeed * 0.8)
      torqueFL = 0.0;
    if (wheelSpeedFR < vehicleSpeed * 0.8)
      torqueFR = 0.0;
    if (wheelSpeedRL < vehicleSpeed * 0.8)
      torqueRL = 0.0;
    if (wheelSpeedRR < vehicleSpeed * 0.8)
      torqueRR = 0.0;
  }

  // === Send CAN Commands ===
  sendMotorCommand(motorFL_ID, torqueFL);
  sendMotorCommand(motorFR_ID, torqueFR);
  sendMotorCommand(motorRL_ID, torqueRL);
  sendMotorCommand(motorRR_ID, torqueRR);

  delay(10); // Loop at 100 Hz
}

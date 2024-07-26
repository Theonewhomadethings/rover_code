/*
This script provides simple RC control of the servos based on the input from the RC receiver, 
with servos 1 and 2 moving synchronously and servo 3 moving independently.
*/

#include <DynamixelShield.h>

// Define IDs and protocol version for Dynamixel servos
const uint8_t DXL_ID1 = 4;  // ID of the first Dynamixel servo
const uint8_t DXL_ID2 = 3;  // ID of the second Dynamixel servo
const uint8_t DXL_ID3 = 5;  // ID of the third Dynamixel servo
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

// Pins for RC receiver inputs
const int throttlePin = 9;
const int rudderPin = 6;

volatile unsigned long throttlePulseStart = 0;
volatile unsigned long rudderPulseStart = 0;
volatile unsigned long throttlePulse = 1500;
volatile unsigned long rudderPulse = 1500;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    // Wait for serial port to connect
  }

  // Initialize Dynamixel Shield
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize the servos
  initializeServo(DXL_ID1);
  initializeServo(DXL_ID2);
  initializeServo(DXL_ID3);

  // Attach interrupts for the RC receiver
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

void initializeServo(uint8_t id) {
  dxl.ping(id);
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(id);
}

void throttleRise() {
  throttlePulseStart = micros();
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleFall, FALLING);
}

void throttleFall() {
  throttlePulse = micros() - throttlePulseStart;
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
}


void loop() {
  // Map throttlePulse (1000 to 2000) to servo positions for the scoop arms
  int armPosition = map(throttlePulse, 1000, 2000, 0, 1023);

  // Set the scoop arm servo positions (synchronously for servos 1 and 2)
  dxl.setGoalPosition(DXL_ID1, armPosition);
  dxl.setGoalPosition(DXL_ID2, armPosition);

   Map rudderPulse (1000 to 2000) to servo position for the scoop tilt
   int scoopPosition = map(rudderPulse, 1000, 2000, 0, 1023);

  // Set the scoop tilt servo position
   dxl.setGoalPosition(DXL_ID3, scoopPosition);

  // Debugging output to monitor the servo positions
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Arm Position: ");
  Serial.print(armPosition);
  Serial.print(", Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Scoop Position: ");
  Serial.println(scoopPosition);

  delay(40); // Small delay for readability and smooth movement
}

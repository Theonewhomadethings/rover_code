#include <DynamixelShield.h>

const uint8_t DXL_ID1 = 1;  // ID of the first Dynamixel servo
const uint8_t DXL_ID2 = 3;  // ID of the second Dynamixel servo
const uint8_t DXL_ID3 = 4;  // ID of the third Dynamixel servo
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// Pins for RC receiver inputs
const int throttlePin = 7;
const int rudderPin = 8;

volatile unsigned long throttlePulseStart = 0;
volatile unsigned long rudderPulseStart = 0;
volatile unsigned long throttlePulse = 1500;
volatile unsigned long rudderPulse = 1500;

unsigned long previousThrottlePulse = 1500;
unsigned long previousRudderPulse = 1500;

// const int MIN_VELOCITY = -1024;  // Define minimum servo velocity
// const int MAX_VELOCITY = 1024;   // Define maximum servo velocity

const int MIN_VELOCITY = -200;  // Define minimum servo velocity (half of full range)
const int MAX_VELOCITY = 200;   // Define maximum servo velocity (half of full range)

void setup() {
  // Initialize serial
  Serial.begin(9600);
  delay(40);   // keep so it doesnt break ther arms

  // Initialize Dynamixel Shield
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize the first servo
  dxl.ping(DXL_ID1);
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);  // Set to velocity control mode
  dxl.torqueOn(DXL_ID1);

  // Initialize the second servo
  dxl.ping(DXL_ID2);
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);  // Set to velocity control mode
  dxl.torqueOn(DXL_ID2);

  // Initialize the third servo
  dxl.ping(DXL_ID3);
  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);  // Set to velocity control mode
  dxl.torqueOn(DXL_ID3);

  // Attach interrupts for the RC receiver
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

void throttleRise() {
  throttlePulseStart = micros();
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleFall, FALLING);
}

void throttleFall() {
  throttlePulse = micros() - throttlePulseStart;
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
}

void rudderRise() {
  rudderPulseStart = micros();
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderFall, FALLING);
}

void rudderFall() {
  rudderPulse = micros() - rudderPulseStart;
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

void loop() {
  // Declare velocity variables outside the conditionals
  int leftArmVelocity, rightArmVelocity, scoopVelocity;

  // Check for change in throttle pulse
  if (throttlePulse != previousThrottlePulse) {
    previousThrottlePulse = throttlePulse;

    // Map throttlePulse (1000 to 2000) to velocity (-1024 to 1024)
    int throttleVelocity = map(throttlePulse, 1000, 2000, MIN_VELOCITY, MAX_VELOCITY);

    // Set the velocity for the first and second servos based on throttle input
    leftArmVelocity = throttleVelocity;
    rightArmVelocity = -throttleVelocity;

    dxl.setGoalVelocity(DXL_ID1, leftArmVelocity);
    dxl.setGoalVelocity(DXL_ID2, rightArmVelocity);
  }

  // Check for change in rudder pulse
  if (rudderPulse != previousRudderPulse) {
    previousRudderPulse = rudderPulse;

    // Map rudderPulse (1000 to 2000) to velocity (-1024 to 1024)
    scoopVelocity = map(rudderPulse, 1000, 2000, MIN_VELOCITY, MAX_VELOCITY);

    // Set the velocity for the third servo based on rudder input
    dxl.setGoalVelocity(DXL_ID3, scoopVelocity);
  }

  // Debugging output to monitor the servo position
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Left Arm Velocity: ");
  Serial.print(leftArmVelocity);
  Serial.print(", Right Arm Velocity: ");
  Serial.print(rightArmVelocity);
  Serial.print(", Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Scoop Velocity: ");
  Serial.println(scoopVelocity);

  delay(40);
}

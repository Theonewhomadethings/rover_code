// Scoop arms only code
// adjust step size and delay for speed of the servo motors

#include <DynamixelShield.h>

const uint8_t DXL_ID1 = 4;  // ID of the first Dynamixel servo
const uint8_t DXL_ID2 = 3;  // ID of the second Dynamixel servo
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// Pins for RC receiver inputs
const int throttlePin = 9;
const int rudderPin = 6;

volatile unsigned long throttlePulseStart = 0;
volatile unsigned long rudderPulseStart = 0;
volatile unsigned long throttlePulse = 1500;
volatile unsigned long rudderPulse = 1500;

int currentPosition1 = 200;  // Start at the middle position for the first servo
int currentPosition2 = 0;  // Start at the middle position for the second servo

const int stepSize = 25;     // Reduced step size for smoother movement
const int delayTime = 40;    // Increased delay for slower movement

void setup() {
  // Initialize serial
  Serial.begin(9600);
  while (!Serial) {
    // wait for serial port to connect.
  }

  // Initialize Dynamixel Shield
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize the first servo
  dxl.ping(DXL_ID1);
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(DXL_ID1);

  // Initialize the second servo
  dxl.ping(DXL_ID2);
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(DXL_ID2);

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
  // Map throttlePulse (1000 to 2000) to servo positions
  int y = map(throttlePulse, 1000, 2000, -512, 512);

  // Update the servo positions based on the throttle input
  if (y < -20) {
    currentPosition1 += stepSize;  // Reduced step size for smoother movement
    currentPosition2 -= stepSize;  // Move both servos simultaneously
  } else if (y > 20) {
    currentPosition1 -= stepSize;  // Reduced step size for smoother movement
    currentPosition2 += stepSize;  // Move both servos simultaneously
  }

  // Set the servo goal positions
  dxl.setGoalPosition(DXL_ID1, currentPosition1);
  dxl.setGoalPosition(DXL_ID2, currentPosition2);

  // Debugging output to monitor the servo position
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Servo 1 Position: ");
  Serial.print(currentPosition1);
  Serial.print(", Servo 2 Position: ");
  Serial.println(currentPosition2);

  delay(delayTime);
}

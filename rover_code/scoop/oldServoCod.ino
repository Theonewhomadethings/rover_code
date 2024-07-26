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

const int minPosition = 0; // Minimum position for servo motors
const int maxPosition = 1024; // Maximum position for servo motors

int targetPosition1 = 512;  // Target position for the first servo
int targetPosition2 = 512;  // Target position for the second servo
int currentPosition1 = 512; // Current position for the first servo
int currentPosition2 = 512; // Current position for the second servo
const int stepSize = 5;     // Step size for smooth movement
const int delayTime = 10;   // Delay between each step for smooth movement

const int zeroPosition1 = 600; // Custom zero position (adjust this value)
const int zeroPosition = 600; // Custom zero position (adjust this value)


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

  // Set initial positions to custom zero position
  currentPosition1 = zeroPosition;
  currentPosition2 = zeroPosition;

  // Set the servo goal positions to custom zero position
  dxl.setGoalPosition(DXL_ID1, currentPosition1);
  dxl.setGoalPosition(DXL_ID2, currentPosition2);

  // Attach interrupts for the RC receiver
  pinMode(throttlePin, INPUT);
  pinMode(rudderPin, INPUT);
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
  int y = map(throttlePulse, 1000, 2000, minPosition, maxPosition);

  // Update target positions based on the throttle input
  if (y < minPosition) {
    targetPosition1 = minPosition;
    targetPosition2 = minPosition;
  } else if (y > maxPosition) {
    targetPosition1 = maxPosition;
    targetPosition2 = maxPosition;
  } else {
    targetPosition1 = y;
    targetPosition2 = y;
  }

  // Smoothly move to the target positions
  if (currentPosition1 < targetPosition1) {
    currentPosition1 -= stepSize;
    if (currentPosition1 > targetPosition1) {
      currentPosition1 = targetPosition1;
    }
  } else if (currentPosition1 > targetPosition1) {
    currentPosition1 += stepSize;
    if (currentPosition1 < targetPosition1) {
      currentPosition1 = targetPosition1;
    }
  }

  if (currentPosition2 < targetPosition2) {
    currentPosition2 += stepSize;
    if (currentPosition2 > targetPosition2) {
      currentPosition2 = targetPosition2;
    }
  } else if (currentPosition2 > targetPosition2) {
    currentPosition2 -= stepSize;
    if (currentPosition2 < targetPosition2) {
      currentPosition2 = targetPosition2;
    }
  }

  // Set the servo goal positions
  dxl.setGoalPosition(DXL_ID1, currentPosition1);
  dxl.setGoalPosition(DXL_ID2, currentPosition2);

  // Debugging output to monitor the servo position
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Target Position 1: ");
  Serial.print(targetPosition1);
  Serial.print(", Current Position 1: ");
  Serial.print(currentPosition1);
  Serial.print(", Target Position 2: ");
  Serial.print(targetPosition2);
  Serial.print(", Current Position 2: ");
  Serial.println(currentPosition2);

  delay(delayTime);
}

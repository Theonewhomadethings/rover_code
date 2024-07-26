// Attempt 1 at getting servo 1 and 2 to work for scoop arms and servo 3 to work for scoop itself
// we try to only constrain the range of motion for the scoop arms and keep the 3rd servo motor for the scoop full range of motion.
// adjust step size and delay for speed of the servo motors

#include <DynamixelShield.h>

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

const int MIN_POSITION = 100;  // Define minimum servo position for scoop arms
const int MAX_POSITION = 924;  // Define maximum servo position for scoop arms
const int ZERO_POSITION1 = 512; // Define zero position for servo 1
const int ZERO_POSITION2 = 512; // Define zero position for servo 2
const int ZERO_POSITION3 = 512; // Define zero position for servo 3

int currentPosition1 = ZERO_POSITION1;  // Start at zero position for the first servo
int currentPosition2 = ZERO_POSITION2;  // Start at zero position for the second servo
int currentPosition3 = ZERO_POSITION3;  // Start at zero position for the third servo

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
  
  // Initialize the third servo
  dxl.ping(DXL_ID3);
  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(DXL_ID3);

  // Attach interrupts for the RC receiver
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);

  // Move servos to zero position smoothly
  moveToZeroPosition();
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

void moveToZeroPosition() {
  // Smoothly move servos to their zero positions
  while (currentPosition1 != ZERO_POSITION1 || currentPosition2 != ZERO_POSITION2 || currentPosition3 != ZERO_POSITION3) {
    if (currentPosition1 < ZERO_POSITION1) {
      currentPosition1 += stepSize;
      if (currentPosition1 > ZERO_POSITION1) currentPosition1 = ZERO_POSITION1;
    } else if (currentPosition1 > ZERO_POSITION1) {
      currentPosition1 -= stepSize;
      if (currentPosition1 < ZERO_POSITION1) currentPosition1 = ZERO_POSITION1;
    }

    if (currentPosition2 < ZERO_POSITION2) {
      currentPosition2 += stepSize;
      if (currentPosition2 > ZERO_POSITION2) currentPosition2 = ZERO_POSITION2;
    } else if (currentPosition2 > ZERO_POSITION2) {
      currentPosition2 -= stepSize;
      if (currentPosition2 < ZERO_POSITION2) currentPosition2 = ZERO_POSITION2;
    }

    if (currentPosition3 < ZERO_POSITION3) {
      currentPosition3 += stepSize;
      if (currentPosition3 > ZERO_POSITION3) currentPosition3 = ZERO_POSITION3;
    } else if (currentPosition3 > ZERO_POSITION3) {
      currentPosition3 -= stepSize;
      if (currentPosition3 < ZERO_POSITION3) currentPosition3 = ZERO_POSITION3;
    }

    currentPosition1 = constrain(currentPosition1, MIN_POSITION, MAX_POSITION);
    currentPosition2 = constrain(currentPosition2, MIN_POSITION, MAX_POSITION);

    dxl.setGoalPosition(DXL_ID1, currentPosition1);
    dxl.setGoalPosition(DXL_ID2, currentPosition2);
    dxl.setGoalPosition(DXL_ID3, currentPosition3);
    delay(delayTime);
  }
}

void loop() {
  // Map throttlePulse (1000 to 2000) to servo positions for the scoop arms
  int y = map(throttlePulse, 1000, 2000, MIN_POSITION, MAX_POSITION);

  // Update the scoop arm servo positions based on the throttle input
  if (y < currentPosition1) {
    currentPosition1 -= stepSize;
    currentPosition2 -= stepSize;
  } else if (y > currentPosition1) {
    currentPosition1 += stepSize;
    currentPosition2 += stepSize;
  }

  // Ensure the scoop arm servo positions stay within the allowed range
  currentPosition1 = constrain(currentPosition1, MIN_POSITION, MAX_POSITION);
  currentPosition2 = constrain(currentPosition2, MIN_POSITION, MAX_POSITION);

  // Set the scoop arm servo goal positions
  dxl.setGoalPosition(DXL_ID1, currentPosition1);
  dxl.setGoalPosition(DXL_ID2, currentPosition2);

  // Map rudderPulse (1000 to 2000) to servo position for the scoop tilt
  int x = map(rudderPulse, 1000, 2000, MIN_POSITION, MAX_POSITION);

  // Update the scoop tilt servo position based on the rudder input
  if (x < currentPosition3) {
    currentPosition3 -= stepSize;
  } else if (x > currentPosition3) {
    currentPosition3 += stepSize;
  }

  // Set the scoop tilt servo goal position
  dxl.setGoalPosition(DXL_ID3, currentPosition3);

  // Debugging output to monitor the servo positions
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Servo 1 Position: ");
  Serial.print(currentPosition1);
  Serial.print(", Servo 2 Position: ");
  Serial.print(currentPosition2);
  Serial.print(", Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Servo 3 Position: ");
  Serial.println(currentPosition3);

  delay(delayTime);
}

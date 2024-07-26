/*
This script provides simple RC control of the servos based on the input from the RC receiver.
It controls two servos (both with ID 3) for the scoop arms and one servo (ID 4) for the scoop tilt.

Components used:
- Arduino board (e.g., Arduino MKR 1010 WiFi)
- Dynamixel servo motors (2 with ID 3 and 1 with ID 4)
- Dynamixel Shield
- RC receiver and transmitter
- RC remote (left joystick for control)

Connections:
- The throttle channel of the RC receiver is connected to pin 9 on the Arduino.
- The rudder channel of the RC receiver is connected to pin 6 on the Arduino.
- The Dynamixel servos are connected via the Dynamixel Shield.

Functionality:
- Initialize the serial communication for debugging.
- Set up the Dynamixel Shield and the servos.
- Use interrupts to read the PWM signals from the throttle and rudder channels.
- Map the received PWM signals to corresponding positions for the servos.
- Continuously update the servo positions based on the RC remote inputs.
*/

#include <DynamixelShield.h>

// Define IDs and protocol version for Dynamixel servos
const uint8_t DXL_ID_ARMS = 3;  // ID for the scoop arms servos
const uint8_t DXL_ID_SCOOP = 4;  // ID for the scoop tilt servo
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

// Pins for RC receiver inputs
const int throttlePin = 9;  // Pin for throttle (up/down) control
const int rudderPin = 6;    // Pin for rudder (left/right) control

volatile unsigned long throttlePulseStart = 0;
volatile unsigned long rudderPulseStart = 0;
volatile unsigned long throttlePulse = 1500;
volatile unsigned long rudderPulse = 1500;

// Current positions of the servos
int currentArmPosition = 512;  // Start at middle position
int currentScoopPosition = 512;  // Start at middle position

// Step size for servo movement
const int stepSize = 40;

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
  initializeServo(DXL_ID_ARMS);
  initializeServo(DXL_ID_SCOOP);

  // Attach interrupts for the RC receiver
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

// Function to initialize a Dynamixel servo
void initializeServo(uint8_t id) {
  dxl.ping(id);                      // Check if the servo is connected
  dxl.torqueOff(id);                 // Turn off the torque to allow setting the mode
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(id);                  // Turn on the torque to enable servo movement
}

// Interrupt service routine called when the throttle signal rises
void throttleRise() {
  throttlePulseStart = micros();       // Record the start time of the pulse
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleFall, FALLING);  // Attach the fall interrupt
}

// Interrupt service routine called when the throttle signal falls
void throttleFall() {
  throttlePulse = micros() - throttlePulseStart;  // Calculate the pulse width
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);   // Reattach the rise interrupt
}

// Interrupt service routine called when the rudder signal rises
void rudderRise() {
  rudderPulseStart = micros();       // Record the start time of the pulse
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderFall, FALLING);  // Attach the fall interrupt
}

// Interrupt service routine called when the rudder signal falls
void rudderFall() {
  rudderPulse = micros() - rudderPulseStart;  // Calculate the pulse width
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);   // Reattach the rise interrupt
}

void loop() {
  // Map throttlePulse (1000 to 2000) to a control range (-1 to 1)
  float throttleControl = map(throttlePulse, 1000, 2000, -100, 100) / 100.0;

  // Map rudderPulse (1000 to 2000) to a control range (-1 to 1)
  float rudderControl = map(rudderPulse, 1000, 2000, -100, 100) / 100.0;

  // Update the scoop arm positions based on the throttle control
  if (throttleControl > 0.1) {
    currentArmPosition += stepSize;
  } else if (throttleControl < -0.1) {
    currentArmPosition -= stepSize;
  }

  // Update the scoop tilt position based on the rudder control
  if (rudderControl > 0.1) {
    currentScoopPosition += stepSize;
  } else if (rudderControl < -0.1) {
    currentScoopPosition -= stepSize;
  }

  // Constrain the positions to the valid range (0 to 1023)
  currentArmPosition = constrain(currentArmPosition, 0, 4095);
  currentScoopPosition = constrain(currentScoopPosition, 0, 4095);

  // Set the scoop arm servo positions (synchronously for both servos with ID 3)
  dxl.setGoalPosition(DXL_ID_ARMS, currentArmPosition);

  // Set the scoop tilt servo position
  dxl.setGoalPosition(DXL_ID_SCOOP, currentScoopPosition);

  // Debugging output to monitor the servo positions
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Arm Position: ");
  Serial.print(currentArmPosition);
  Serial.print(", Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Scoop Position: ");
  Serial.println(currentScoopPosition);

  delay(40); // Small delay for readability and smooth movement
}

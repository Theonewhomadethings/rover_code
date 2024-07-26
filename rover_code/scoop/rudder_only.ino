
/*
This script provides a simple RC control for a single Dynamixel servo motor (ID3), 
based on input from an RC receiver's rudder channel. The rudder channel corresponds 
to the left and right movements on the left joystick of the RC remote. 

The purpose of this script is to test the control of the third servo motor independently, 
by mapping the pulse width modulation (PWM) signal received from the RC receiver to the 
servo's position. The script initializes the necessary hardware components, 
sets up the servo, and continuously reads the PWM signal from the rudder channel to update 
the servo's position accordingly.

Components used:
- Arduino board (e.g., Arduino MKR 1010 WiFi)
- Dynamixel servo motor (ID3)
- Dynamixel Shield
- RC receiver and transmitter
- RC remote (left joystick for control)

Connections:
- The rudder channel of the RC receiver is connected to pin 6 on the Arduino.
- The Dynamixel servo is connected via the Dynamixel Shield.

Functionality:
- Initialize the serial communication for debugging.
- Set up the Dynamixel Shield and the third servo motor.
- Use interrupts to read the PWM signal from the rudder channel.
- Map the received PWM signal to a corresponding position for the servo.
- Continuously update the servo position based on the rudder input.

The goal is to ensure that the left and right movements on the joystick correctly control 
the position of the third servo motor, allowing for troubleshooting and verification of 
basic servo control.
*/
#include <DynamixelShield.h>

// Define the ID and protocol version for the third Dynamixel servo
const uint8_t DXL_ID3 = 5;  // ID of the third Dynamixel servo
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

// Pin for RC receiver input (rudder)
const int rudderPin = 6;

volatile unsigned long rudderPulseStart = 0;
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

  // Initialize the third servo
  initializeServo(DXL_ID3);

  // Attach interrupt for the RC receiver (rudder)
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

// Function to initialize a Dynamixel servo
void initializeServo(uint8_t id) {
  dxl.ping(id);                      // Check if the servo is connected
  dxl.torqueOff(id);                 // Turn off the torque to allow setting the mode
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);  // Set to extended position mode
  dxl.torqueOn(id);                  // Turn on the torque to enable servo movement
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
  // Map rudderPulse (1000 to 2000) to servo position for the scoop tilt
  int scoopPosition = map(rudderPulse, 1000, 2000, 0, 1023);

  // Set the scoop tilt servo position
  dxl.setGoalPosition(DXL_ID3, scoopPosition);

  // Debugging output to monitor the servo position
  Serial.print("Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Scoop Position: ");
  Serial.println(scoopPosition);

  delay(40); // Small delay for readability and smooth movement
}

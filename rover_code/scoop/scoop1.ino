#include <DynamixelShield.h>

// Define IDs and protocol version for Dynamixel servos
const uint8_t DXL_ID_ARM_LEFT = 1;  // ID for the left arm servo
const uint8_t DXL_ID_ARM_RIGHT = 3;  // ID for the right arm servo
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

// Current positions of the servos (angle in degrees)
float currentArmAngle = 0;  // Start at 0 degrees
float currentScoopAngle = 0;  // Start at 0 degrees

// Step size for servo movement (degrees)
const float stepSize = 5.0;

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
  initializeServo(DXL_ID_ARM_LEFT);
  initializeServo(DXL_ID_ARM_RIGHT);
  initializeServo(DXL_ID_SCOOP);

  // Attach interrupts for the RC receiver
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleRise, RISING);
  attachInterrupt(digitalPinToInterrupt(rudderPin), rudderRise, RISING);
}

// Function to initialize a Dynamixel servo
void initializeServo(uint8_t id) {
  dxl.ping(id);                      // Check if the servo is connected
  dxl.torqueOff(id);                 // Turn off the torque to allow setting the mode
  dxl.setOperatingMode(id, OP_PWM);  // Set to PWM control mode
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

  // Update the arm positions based on the throttle control
  if (throttleControl > 0.1) {
    currentArmAngle += stepSize;
  } else if (throttleControl < -0.1) {
    currentArmAngle -= stepSize;
  }

  // Update the scoop tilt position based on the rudder control
  if (rudderControl > 0.1) {
    currentScoopAngle += stepSize;
  } else if (rudderControl < -0.1) {
    currentScoopAngle -= stepSize;
  }

  // Constrain the positions to the valid range (-90 to 90 degrees for a total of 180 degrees)
  currentArmAngle = constrain(currentArmAngle, -90.0, 90.0);
  currentScoopAngle = constrain(currentScoopAngle, -90.0, 90.0);

  // Convert angles to PWM values (assuming -90 to 90 degrees maps to -885 to 885 in PWM mode)
  int leftArmPWM = map(currentArmAngle, -90, 90, -885, 885);
  int rightArmPWM = map(-currentArmAngle, -90, 90, -885, 885); // Move in opposite direction
  int scoopPWM = map(currentScoopAngle, -90, 90, -885, 885);

  // Set the scoop arm servo positions (IDs 1 and 3)
  dxl.setGoalPWM(DXL_ID_ARM_LEFT, leftArmPWM);
  dxl.setGoalPWM(DXL_ID_ARM_RIGHT, rightArmPWM);

  // Set the scoop tilt servo position
  dxl.setGoalPWM(DXL_ID_SCOOP, scoopPWM);

  // Debugging output to monitor the servo positions
  Serial.print("Throttle Pulse: ");
  Serial.print(throttlePulse);
  Serial.print(", Arm Angle: ");
  Serial.print(currentArmAngle);
  Serial.print(", Left Arm PWM: ");
  Serial.print(leftArmPWM);
  Serial.print(", Right Arm PWM: ");
  Serial.print(rightArmPWM);
  Serial.print(", Rudder Pulse: ");
  Serial.print(rudderPulse);
  Serial.print(", Scoop Angle: ");
  Serial.print(currentScoopAngle);
  Serial.print(", Scoop PWM: ");
  Serial.println(scoopPWM);

  delay(40); // Small delay for readability and smooth movement
}

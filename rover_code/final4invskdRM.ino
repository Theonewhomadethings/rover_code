#include <DynamixelShield.h>

#define RCPinFWD 7  // Pin for forward/backward signal from RC receiver
#define RCPinTURN 8  // Pin for left/right signal from RC receiver
#define IN1 4 // Digital pin for direction control
#define IN2 5 // PWM pin for speed control
#define IN3 2 // Digital pin for direction control
#define IN4 3 // PWM pin for speed control

// Variables to store pulse width timing for forward/backward and left/right control
volatile long StartTimeFWD = 0;
volatile long CurrentTimeFWD = 0;
volatile long PulsesFWD = 0;
int PulseWidthFWD = 0;

volatile long StartTimeTURN = 0;
volatile long CurrentTimeTURN = 0;
volatile long PulsesTURN = 0;
int PulseWidthTURN = 0;

// Deadband threshold
const int deadbandThreshold = 0 ; // Adjusted for a practical deadband

const uint8_t DXL_ID1 = 1;  // ID of the first Dynamixel servo
const uint8_t DXL_ID2 = 3;  // ID of the second Dynamixel servo
const uint8_t DXL_ID3 = 4;  // ID of the third Dynamixel servo
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// Pins for RC receiver inputs
const int throttlePin = 9;
const int rudderPin = 6;

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
  delay(100);   // keep so it doesnt break ther arms

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

  pinMode(RCPinFWD, INPUT_PULLUP);  // Set pin for forward/backward signal as input with pull-up resistor
  pinMode(RCPinTURN, INPUT_PULLUP); // Set pin for left/right signal as input with pull-up resistor

  // Attach interrupts to capture PWM signals from RC receiver
  attachInterrupt(digitalPinToInterrupt(RCPinFWD), PulseTimerFWD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTURN), PulseTimerTURN, CHANGE);

  // Set motor driver control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
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

  // Read pulse widths from RC receiver
  if (PulsesFWD < 2000) {
    PulseWidthFWD = PulsesFWD;
  }
  if (PulsesTURN < 2000) {
    PulseWidthTURN = PulsesTURN;
  }

  // Map the PWM values to motor speed and direction
  int speedFWD = map(PulseWidthFWD, 1000, 2000, -255, 255);
  int turn = map(PulseWidthTURN, 1000, 2000, -255, 255);

  // Apply deadband
  if (abs(speedFWD) == deadbandThreshold) {
    speedFWD = 0;
  }
  if (abs(turn) == deadbandThreshold) {
    turn = 0;
  }

  // Debugging statements
  Serial.print("PulsesFWD: ");
  Serial.print(PulsesFWD);
  Serial.print(", PulseWidthFWD: ");
  Serial.print(PulseWidthFWD);
  Serial.print(", speedFWD: ");
  Serial.println(speedFWD);
  
  Serial.print("PulsesTURN: ");
  Serial.print(PulsesTURN);
  Serial.print(", PulseWidthTURN: ");
  Serial.print(PulseWidthTURN);
  Serial.print(", turn: ");
  Serial.println(turn);

  // Control motors based on PWM values
  controlMotors(speedFWD, turn);

  delay(100); // Small delay for readability
}

// Interrupt service routine for forward/backward signal
void PulseTimerFWD() {
  CurrentTimeFWD = micros();
  if (CurrentTimeFWD > StartTimeFWD) {
    PulsesFWD = CurrentTimeFWD - StartTimeFWD;
    StartTimeFWD = CurrentTimeFWD;
  }
}

// Interrupt service routine for left/right signal
void PulseTimerTURN() {
  CurrentTimeTURN = micros();
  if (CurrentTimeTURN > StartTimeTURN) {
    PulsesTURN = CurrentTimeTURN - StartTimeTURN;
    StartTimeTURN = CurrentTimeTURN;
  }
}

// Function to control the motors based on speed and turn values
void controlMotors(int speedFWD, int turn) {

    // Invert the turn value
    turn = -turn;
  
  int leftMotorSpeed = speedFWD + turn;
  int rightMotorSpeed = speedFWD - turn;

  // Constrain speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Debugging statements for motor control
  Serial.print("leftMotorSpeed: ");
  Serial.print(leftMotorSpeed);
  Serial.print(", rightMotorSpeed: ");
  Serial.println(rightMotorSpeed);

  // Control left motors
  if (leftMotorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    analogWrite(IN2, leftMotorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, -leftMotorSpeed);
  }

  // Control right motors
  if (rightMotorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    analogWrite(IN4, rightMotorSpeed);
  } else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, -rightMotorSpeed);
  }
}

/*

  skid steering for MKR  arduino board
  version 1
	
    when we press up a little on our RC remote joystick the motors go full speed
    when we press up all the way on our rc remote joystick the motors move a little 

    we expect the behaiour

    when we press up a little on our RC remote joystick the motors move a little
    when we press up all the way on our rc remote joystick the motors go full speed forward

    our hardware set up is:
    - Arduino mkr 1010 wifi
    - 4x 12v dc motor
    - 2x L298N motor drive boards
    - RC Reciever (flight based throttle, rudder etc)
    - RC remote + transmitter
*/
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
const int deadbandThreshold = -120 ; // Adjusted for a practical deadband

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
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

void loop() {
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
  
#define RCPinFWD 2  // Pin for forward/backward signal from RC receiver
#define IN1 7       // Digital pin for direction control
#define IN2 6       // PWM pin for speed control
#define IN3 4       // Digital pin for direction control
#define IN4 5       // PWM pin for speed control

// Variables to store pulse width timing for forward/backward control
volatile long StartTimeFWD = 0;
volatile long CurrentTimeFWD = 0;
volatile long PulsesFWD = 0;
int PulseWidthFWD = 0;

// Deadband threshold
const int deadbandThreshold = 30;

void setup() {
  Serial.begin(9600);               // Initialize serial communication at 9600 baud rate
  pinMode(RCPinFWD, INPUT_PULLUP);  // Set pin for forward/backward signal as input with pull-up resistor

  // Attach interrupts to capture PWM signals from RC receiver
  attachInterrupt(digitalPinToInterrupt(RCPinFWD), PulseTimerFWD, CHANGE);

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

  // Map the PWM values to motor speed and direction
  int speedFWD = map(PulseWidthFWD, 1000, 2000, -255, 255);

  // Apply deadband
  if (abs(speedFWD) < deadbandThreshold) {
    speedFWD = 0;
  }

  // Control motors based on PWM values
  controlMotors(speedFWD);

  // and direction
  Serial.print("PulseWidthFWD: ");
  Serial.print(PulseWidthFWD);
  Serial.print(", SpeedFWD: ");
  Serial.println(speedFWD);

  delay(100);  // Small delay for readability
}

// Interrupt service routine for forward/backward signal
void PulseTimerFWD() {
  CurrentTimeFWD = micros();
  if (CurrentTimeFWD > StartTimeFWD) {
    PulsesFWD = CurrentTimeFWD - StartTimeFWD;
    StartTimeFWD = CurrentTimeFWD;
  }
}

// Function to control the motors based on speed values
void controlMotors(int speedFWD) {
  if (speedFWD > 0) {
    // Move all motors forward
    digitalWrite(IN1, LOW);  // Assuming LOW is forward
    digitalWrite(IN3, LOW);  // Assuming LOW is forward
    analogWrite(IN2, speedFWD);
    analogWrite(IN4, speedFWD);
  } else if (speedFWD < 0) {
    // Move all motors backward
    digitalWrite(IN1, HIGH);      // Assuming HIGH is backward
    digitalWrite(IN3, HIGH);      // Assuming HIGH is backward
    analogWrite(IN2, -speedFWD);  // Ensure the PWM value is positive
    analogWrite(IN4, -speedFWD);  // Ensure the PWM value is positive
  } else {
    // Stop all motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN3, LOW);
    analogWrite(IN2, 0);
    analogWrite(IN4, 0);
  }
}

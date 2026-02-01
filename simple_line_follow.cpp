#include <Servo.h>

// ===== L298N Motor Driver Pins =====
#define ENA 11
#define ENB 10
#define IN1 13
#define IN2 12
#define IN3 9
#define IN4 8

// ===== Color Sensor Pins =====
#define SENSOR_OUT A1
#define SENSOR_S0 2
#define SENSOR_S1 3
#define SENSOR_S2 4
#define SENSOR_S3 7

// ===== Servo Pin =====
#define SERVO_PIN 6

// ===== Ultrasonic Sensor Pins =====
#define TRIGGER 5
#define ECHO A2

// ===== Color Thresholds =====
// Red Line: R:23-26  G:95-110  B:75-85
// Dark Red: R:33-37  G:107-115  B:91-100 (treat as line)
// White: R:17  G:18  B:16
#define LINE_RED_MIN 23
#define LINE_RED_MAX 37      // Extended to include dark red
#define LINE_GREEN_MIN 95
#define LINE_GREEN_MAX 115   // Extended to include dark red
#define LINE_BLUE_MIN 75
#define LINE_BLUE_MAX 100    // Extended to include dark red

// Average values for mapping (using midpoint of red + dark red range)
#define LINE_RED 30
#define LINE_GREEN 105
#define LINE_BLUE 88

#define WHITE_RED 17
#define WHITE_GREEN 18
#define WHITE_BLUE 16

// Motor speed (0-255)
#define BASE_SPEED 100       // Slower base speed for sharp turns
#define MAX_SPEED 200
#define MIN_SPEED 0

// ===== PD Control Parameters =====
// Tune these values based on your robot's behavior
#define KP 3.5      // Proportional gain - increased for sharp 90 degree turns
#define KD 2.0      // Derivative gain - helps anticipate and smooth sharp turns

// ===== Global Variables =====
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

// PD Control variables
float error = 0;          // Current error (negative = black/left, positive = white/right)
float lastError = 0;      // Previous error for derivative calculation
float derivative = 0;     // Rate of change of error
float pdOutput = 0;       // Combined PD output

// Start sequence flag
bool startSequenceComplete = false;

Servo servoMotor;

void setup() {
  Serial.begin(9600);
  
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Color sensor pins
  pinMode(SENSOR_OUT, INPUT);
  pinMode(SENSOR_S0, OUTPUT);
  pinMode(SENSOR_S1, OUTPUT);
  pinMode(SENSOR_S2, OUTPUT);
  pinMode(SENSOR_S3, OUTPUT);
  
  // Ultrasonic pins
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Initialize color sensor
  digitalWrite(SENSOR_S0, HIGH);
  digitalWrite(SENSOR_S1, LOW);
  
  // Initialize servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(11); // Center position
  
  delay(1000);
  Serial.println("Line Follower Initialized");
}

void loop() {
  // ===== HARDCODED START SEQUENCE =====
  // Run once at the beginning to pass the sharp 90 degree turns
  if (!startSequenceComplete) {
    Serial.println("Starting hardcoded sequence...");
    
    // Step 1: Go forward ~25cm
    Serial.println("Forward 25cm");
    driveForward(BASE_SPEED, 1000);  // Adjust time as needed
    
    // Step 2: Turn 90 degrees LEFT
    Serial.println("Turn 90 LEFT");
    turnRight90();
    
    // Step 3: Go forward ~25cm
    Serial.println("Forward 25cm");
    driveForward(BASE_SPEED, 1500);  // Adjust time as needed
    
    // Step 4: Turn 90 degrees RIGHT
    Serial.println("Turn 90 RIGHT");
    turnLeft90();
    
    Serial.println("Start sequence complete, switching to line follow");
    startSequenceComplete = true;
    return;
  }
  
  // ===== PD LINE FOLLOWING =====
  readColorFast();  // Use faster color reading
  
  // Calculate error based on color sensor reading
  // Red line detection - use green channel as it has the biggest difference
  // Red line has high green (~102), white has low green (~14)
  // Red line = negative error = turn LEFT
  // White = positive error = turn RIGHT
  
  // Map green frequency to error value
  // Red line (~102) maps to -100 (turn left)
  // White (~14) maps to +100 (turn right)
  error = map(greenValue, WHITE_GREEN, LINE_GREEN, 100, -100);
  error = constrain(error, -100, 100);
  
  // Calculate derivative (rate of change)
  derivative = error - lastError;
  
  // Calculate PD output
  pdOutput = (KP * error) + (KD * derivative);
  
  // Apply PD output to motor speeds
  // Positive pdOutput = turn right (slow down right motor)
  // Negative pdOutput = turn left (slow down left motor)
  int leftSpeed = BASE_SPEED - pdOutput;
  int rightSpeed = BASE_SPEED + pdOutput;
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  // Apply motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  // Debug output
  Serial.print("G: ");
  Serial.print(greenValue);
  Serial.print(" Err: ");
  Serial.print(error);
  Serial.print(" D: ");
  Serial.print(derivative);
  Serial.print(" PD: ");
  Serial.print(pdOutput);
  Serial.print(" L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
  
  // Store error for next iteration
  lastError = error;
  
  delay(20);  // Small delay for stability
}

// ===== Color Sensor Functions =====
// Fast color reading for responsive PD control
void readColorFast() {
  // 1. Read RED (S2 = LOW, S3 = LOW)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  redValue = pulseIn(SENSOR_OUT, LOW, 10000);  // 10ms timeout
  
  // 2. Read GREEN (S2 = HIGH, S3 = HIGH)
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  greenValue = pulseIn(SENSOR_OUT, LOW, 10000);
  
  // 3. Read BLUE (S2 = LOW, S3 = HIGH)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  blueValue = pulseIn(SENSOR_OUT, LOW, 10000);
}

// Original slower reading for calibration
void readColor() {
  // 1. Read RED (S2 = LOW, S3 = LOW)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  redValue = pulseIn(SENSOR_OUT, LOW);
  Serial.print("R: ");
  Serial.print(redValue);
  Serial.print("  ");
  delay(100);
  
  // 2. Read GREEN (S2 = HIGH, S3 = HIGH)
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  greenValue = pulseIn(SENSOR_OUT, LOW);
  Serial.print("G: ");
  Serial.print(greenValue);
  Serial.print("  ");
  delay(100);
  
  // 3. Read BLUE (S2 = LOW, S3 = HIGH)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  blueValue = pulseIn(SENSOR_OUT, LOW);
  Serial.print("B: ");
  Serial.println(blueValue);
  delay(100);
}

bool isBlack() {
  // Check if color is closer to black values (raw frequencies)
  // You may need to calibrate these frequency thresholds based on your sensor readings
  // Lower frequency = darker color
  
  // Typical black frequencies are higher than white
  // Black should have frequency > ~100
  return (redValue > 50 && greenValue > 50 && blueValue > 50);
}

bool isWhite() {
  // Check if color is closer to white values (raw frequencies)
  // White should have frequency < ~50
  return (redValue < 50 && greenValue < 50 && blueValue < 50);
}

// ===== Motor Control Functions =====
void moveForward(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void moveLeft(int speed) {
  // Left motor slow
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed / 2);
  
  // Right motor fast
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void moveRight(int speed) {
  // Left motor fast
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor slow
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed / 2);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===== Hardcoded Movement Functions =====
// Drive forward for a specified duration (ms)
void driveForward(int speed, int duration) {
  moveForward(speed);
  delay(duration);
  stopMotors();
  delay(100);  // Brief pause
}

// Turn 90 degrees LEFT (pivot turn)
// Adjust TURN_TIME_90 based on your robot - start with 500ms and tune
#define TURN_TIME_90 700
#define TURN_SPEED_90 150

void turnLeft90() {
  // Left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, TURN_SPEED_90);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, TURN_SPEED_90);
  
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);  // Brief pause
}

// Turn 90 degrees RIGHT (pivot turn)
void turnRight90() {
  // Left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, TURN_SPEED_90);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, TURN_SPEED_90);
  
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);  // Brief pause
}

// Set individual motor speeds for PD control
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

// ===== Ultrasonic Sensor Function =====
long getDistance() {
  // Send trigger pulse
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  
  // Read echo time
  long duration = pulseIn(ECHO, HIGH, 30000);
  
  // Convert to distance (cm)
  long distance = duration / 58;
  
  return distance;
}

// ===== Servo Control Function =====
void setServoAngle(int angle) {
  servoMotor.write(angle);
}
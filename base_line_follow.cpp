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
// Black: R:135-165  G:135-170  B:130-150
#define BLACK_RED_MIN 135
#define BLACK_RED_MAX 165
#define BLACK_GREEN_MIN 135
#define BLACK_GREEN_MAX 170
#define BLACK_BLUE_MIN 130
#define BLACK_BLUE_MAX 150

// White: R:17  G:18  B:16
#define WHITE_RED 17
#define WHITE_GREEN 18
#define WHITE_BLUE 16

// Red Line: R:23-26  G:95-110  B:75-85
#define RED_RED_MIN 23
#define RED_RED_MAX 26
#define RED_GREEN_MIN 95
#define RED_GREEN_MAX 110
#define RED_BLUE_MIN 75
#define RED_BLUE_MAX 85

// Blue (STOP): R:76-81  G:48-51  B:27-30
#define BLUE_RED_MIN 76
#define BLUE_RED_MAX 81
#define BLUE_GREEN_MIN 48
#define BLUE_GREEN_MAX 51
#define BLUE_BLUE_MIN 27
#define BLUE_BLUE_MAX 30

// Motor speed (0-255)
#define BASE_SPEED 100
#define MAX_SPEED 200
#define MIN_SPEED 0

// ===== PD Control Parameters =====
#define KP 3.5      // Proportional gain
#define KD 2.0      // Derivative gain

// ===== Global Variables =====
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

// PD Control variables
float error = 0;
float lastError = 0;
float derivative = 0;
float pdOutput = 0;

// Robot state
bool stopped = false;

// Color confirmation counters (require multiple readings to confirm blue)
#define COLOR_CONFIRM_COUNT 5    // Number of consecutive readings required
int blueConfirmCount = 0;

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
  
  // Initialize color sensor (20% frequency scaling)
  digitalWrite(SENSOR_S0, HIGH);
  digitalWrite(SENSOR_S1, LOW);
  
  // Initialize servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(11); // Center position
  
  delay(1000);
  Serial.println("Base Line Follower Initialized");
  Serial.println("Follows: Black/White and Red/White lines");
  Serial.println("Stops on: Blue");
}

void loop() {
  // If stopped, stay stopped
  if (stopped) {
    stopMotors();
    delay(100);
    return;
  }
  
  // Read color sensor
  readColorFast();
  
  // ===== CHECK FOR STOP CONDITIONS (with confirmation) =====
  // Require multiple consecutive readings to confirm blue
  // This prevents false positives during black/white transitions
  
  if (isBlue()) {
    blueConfirmCount++;
    if (blueConfirmCount >= COLOR_CONFIRM_COUNT) {
      Serial.println("BLUE confirmed - STOPPING!");
      stopMotors();
      stopped = true;
      
      // ===== UNCOMMENT BELOW FOR HARDCODED TURN SEQUENCE =====
      // // Step 1: Set servo to 5 degrees initially
      // Serial.println("Setting servo to 5 degrees");
      // servoMotor.write(5);
      // delay(300);  // Wait for servo to move
      // 
      // // Step 2: Turn 90 degrees to the right
      // Serial.println("Turning RIGHT 90 degrees");
      // turnRight90();
      // 
      // // Step 3: Set servo to 11 degrees after turn
      // Serial.println("Setting servo to 11 degrees");
      // servoMotor.write(11);
      // delay(300);  // Wait for servo to move
      // 
      // Serial.println("Blue sequence complete");
      // ===== END HARDCODED TURN SEQUENCE =====
      
      return;
    }
  } else {
    // Not blue - reset counter
    blueConfirmCount = 0;
  }
  
  // ===== DETERMINE LINE TYPE AND CALCULATE ERROR =====
  // Check if we're on a black line or red line
  
  if (isBlack()) {
    // Black line detected - we're on the line
    // Black has high values (~150), white has low values (~17)
    // Use green channel for consistency
    // Black (~150) = on line = negative error
    // White (~18) = off line = positive error
    error = map(greenValue, WHITE_GREEN, 150, 100, -100);
  } 
  else if (isRed()) {
    // Red line detected - we're on the line
    // Red line has green ~102, white has green ~18
    error = map(greenValue, WHITE_GREEN, 102, 100, -100);
  }
  else {
    // Likely on white or transitioning - use general mapping
    // Higher green = more likely on line
    error = map(greenValue, WHITE_GREEN, 150, 100, -100);
  }
  
  error = constrain(error, -100, 100);
  
  // Calculate derivative (rate of change)
  derivative = error - lastError;
  
  // Calculate PD output
  pdOutput = (KP * error) + (KD * derivative);
  
  // Apply PD output to motor speeds
  int leftSpeed = BASE_SPEED - pdOutput;
  int rightSpeed = BASE_SPEED + pdOutput;
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  // Apply motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  // Debug output
  Serial.print("R:");
  Serial.print(redValue);
  Serial.print(" G:");
  Serial.print(greenValue);
  Serial.print(" B:");
  Serial.print(blueValue);
  Serial.print(" | Err:");
  Serial.print(error);
  Serial.print(" L:");
  Serial.print(leftSpeed);
  Serial.print(" R:");
  Serial.println(rightSpeed);
  
  // Store error for next iteration
  lastError = error;
  
  delay(20);
}

// ===== Color Detection Functions =====
bool isBlack() {
  return (redValue >= BLACK_RED_MIN && redValue <= BLACK_RED_MAX &&
          greenValue >= BLACK_GREEN_MIN && greenValue <= BLACK_GREEN_MAX &&
          blueValue >= BLACK_BLUE_MIN && blueValue <= BLACK_BLUE_MAX);
}

bool isRed() {
  return (redValue >= RED_RED_MIN && redValue <= RED_RED_MAX &&
          greenValue >= RED_GREEN_MIN && greenValue <= RED_GREEN_MAX &&
          blueValue >= RED_BLUE_MIN && blueValue <= RED_BLUE_MAX);
}

bool isBlue() {
  return (redValue >= BLUE_RED_MIN && redValue <= BLUE_RED_MAX &&
          greenValue >= BLUE_GREEN_MIN && greenValue <= BLUE_GREEN_MAX &&
          blueValue >= BLUE_BLUE_MIN && blueValue <= BLUE_BLUE_MAX);
}

bool isWhite() {
  // White has very low frequency values
  return (redValue < 30 && greenValue < 30 && blueValue < 30);
}

// ===== Color Sensor Functions =====
void readColorFast() {
  // Read RED (S2 = LOW, S3 = LOW)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  redValue = pulseIn(SENSOR_OUT, LOW, 10000);
  
  // Read GREEN (S2 = HIGH, S3 = HIGH)
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  greenValue = pulseIn(SENSOR_OUT, LOW, 10000);
  
  // Read BLUE (S2 = LOW, S3 = HIGH)
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  blueValue = pulseIn(SENSOR_OUT, LOW, 10000);
}

// ===== Motor Control Functions =====
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===== Turn Functions =====
#define TURN_TIME_90 650
#define TURN_SPEED_90 150

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
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);
  long distance = duration / 58;
  
  return distance;
}

// ===== Servo Control Function =====
void setServoAngle(int angle) {
  servoMotor.write(angle);
}

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

// ===== Servo & Ultrasonic Pins =====
#define SERVO_PIN 6
#define TRIGGER 5
#define ECHO A2

// ===== Color Thresholds (Keep your calibrated values here) =====
#define BLACK_RED_MIN 135
#define BLACK_RED_MAX 165
#define BLACK_GREEN_MIN 135
#define BLACK_GREEN_MAX 170
#define BLACK_BLUE_MIN 130
#define BLACK_BLUE_MAX 150

#define WHITE_RED 17
#define WHITE_GREEN 18
#define WHITE_BLUE 16

#define RED_RED_MIN 23
#define RED_RED_MAX 26
#define RED_GREEN_MIN 95
#define RED_GREEN_MAX 110
#define RED_BLUE_MIN 75
#define RED_BLUE_MAX 85

#define BLUE_RED_MIN 76
#define BLUE_RED_MAX 81
#define BLUE_GREEN_MIN 48
#define BLUE_GREEN_MAX 51
#define BLUE_BLUE_MIN 27
#define BLUE_BLUE_MAX 30

#define GREEN_RED_MIN 50
#define GREEN_RED_MAX 70
#define GREEN_GREEN_MIN 30
#define GREEN_GREEN_MAX 50
#define GREEN_BLUE_MIN 40
#define GREEN_BLUE_MAX 60

// Motor speed (0-255)
#define BASE_SPEED 70
#define MAX_SPEED 125
#define MIN_SPEED 0
#define KP 3.5
#define KD 2.0

// ===== Global Variables =====
int redValue = 0, greenValue = 0, blueValue = 0;
float error = 0, lastError = 0, derivative = 0, pdOutput = 0;
unsigned long lastSendTime = 0;
const int sendInterval = 800; // Time between AI commentary updates

enum RobotState { STATE_FOLLOW_LINE, STATE_BLUE_DETECTED_1, STATE_FOLLOW_TO_BLACK, STATE_ROTATE_TO_GREEN, STATE_FOLLOW_GREEN, STATE_BLUE_DETECTED_2, STATE_RETURN_PATH, STATE_DONE };
RobotState currentState = STATE_FOLLOW_LINE;
int blueDetectCount = 0, blueConfirmCount = 0, blackConfirmCount = 0, greenConfirmCount = 0;
#define COLOR_CONFIRM_COUNT 5

Servo servoMotor;

void setup() {
  Serial.begin(115200); // Higher speed for AI processing
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(SENSOR_OUT, INPUT); pinMode(SENSOR_S0, OUTPUT); pinMode(SENSOR_S1, OUTPUT); pinMode(SENSOR_S2, OUTPUT); pinMode(SENSOR_S3, OUTPUT);
  pinMode(TRIGGER, OUTPUT); pinMode(ECHO, INPUT);
  digitalWrite(SENSOR_S0, HIGH); digitalWrite(SENSOR_S1, LOW);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(11);
  delay(1000);
}

void loop() {
  readColorFast();
  
  // Send data to Python for commentary
  if (millis() - lastSendTime > sendInterval) {
    sendDataToPython();
    lastSendTime = millis();
  }

  switch (currentState) {
    case STATE_FOLLOW_LINE:
      if (isBlue()) {
        blueConfirmCount++;
        if (blueConfirmCount >= COLOR_CONFIRM_COUNT) {
          stopMotors(); delay(1000); turn180();
          blueConfirmCount = 0; blueDetectCount++;
          currentState = STATE_FOLLOW_TO_BLACK;
          return;
        }
      } else { blueConfirmCount = 0; }
      doLineFollowing();
      break;

    case STATE_FOLLOW_TO_BLACK:
      if (isBlack()) {
        blackConfirmCount++;
        if (blackConfirmCount >= COLOR_CONFIRM_COUNT) {
          stopMotors(); delay(200); blackConfirmCount = 0;
          currentState = STATE_ROTATE_TO_GREEN;
          return;
        }
      } else { blackConfirmCount = 0; }
      doLineFollowing();
      break;

    case STATE_ROTATE_TO_GREEN:
      rotateSlowRight();
      if (isGreen()) {
        greenConfirmCount++;
        if (greenConfirmCount >= COLOR_CONFIRM_COUNT) {
          stopMotors(); delay(100); greenConfirmCount = 0;
          currentState = STATE_FOLLOW_GREEN;
          return;
        }
      } else { greenConfirmCount = 0; }
      break;

    case STATE_FOLLOW_GREEN:
      if (isBlue()) {
        blueConfirmCount++;
        if (blueConfirmCount >= COLOR_CONFIRM_COUNT) {
          stopMotors(); delay(1000); turn180();
          blueConfirmCount = 0; blueDetectCount++;
          currentState = STATE_RETURN_PATH;
          return;
        }
      } else { blueConfirmCount = 0; }
      doLineFollowingGreen();
      break;

    case STATE_RETURN_PATH:
      doLineFollowing();
      break;

    case STATE_DONE:
      stopMotors();
      break;
  }
  delay(20);
}

void sendDataToPython() {
  long distance = getDistance();
  String colorLabel = "NEUTRAL";
  if (isBlack()) colorLabel = "BLACK_LINE";
  else if (isRed()) colorLabel = "RED_DANGER";
  else if (isBlue()) colorLabel = "BLUE_ZONE";
  else if (isGreen()) colorLabel = "GREEN_PATH";
  
  Serial.print(distance);
  Serial.print(",");
  Serial.println(colorLabel);
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

bool isGreen() {
  return (redValue >= GREEN_RED_MIN && redValue <= GREEN_RED_MAX &&
          greenValue >= GREEN_GREEN_MIN && greenValue <= GREEN_GREEN_MAX &&
          blueValue >= GREEN_BLUE_MIN && blueValue <= GREEN_BLUE_MAX);
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

// Turn 180 degrees (two 90 degree turns)
void turn180() {
  // Left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, TURN_SPEED_90);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, TURN_SPEED_90);
  
  delay(TURN_TIME_90 * 2);  // Double the time for 180
  stopMotors();
  delay(100);
}

// Slow rotation to the right for finding colors
#define ROTATE_SPEED 80
void rotateSlowRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, ROTATE_SPEED);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, ROTATE_SPEED);
}

// Standard line following logic
void doLineFollowing() {
  // Determine error based on line type
  if (isBlack()) {
    error = map(greenValue, WHITE_GREEN, 150, 100, -100);
  } 
  else if (isRed()) {
    error = map(greenValue, WHITE_GREEN, 102, 100, -100);
  }
  else {
    error = map(greenValue, WHITE_GREEN, 150, 100, -100);
  }
  
  error = constrain(error, -100, 100);
  derivative = error - lastError;
  pdOutput = (KP * error) + (KD * derivative);
  
  int leftSpeed = BASE_SPEED - pdOutput;
  int rightSpeed = BASE_SPEED + pdOutput;
  
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
  lastError = error;
}

// Line following for green line
void doLineFollowingGreen() {
  // Green line - use green channel midpoint (~40)
  error = map(greenValue, WHITE_GREEN, 40, 100, -100);
  error = constrain(error, -100, 100);
  
  derivative = error - lastError;
  pdOutput = (KP * error) + (KD * derivative);
  
  int leftSpeed = BASE_SPEED - pdOutput;
  int rightSpeed = BASE_SPEED + pdOutput;
  
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
  lastError = error;
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
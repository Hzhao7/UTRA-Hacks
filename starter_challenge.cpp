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

// Green Line: TODO - calibrate these values!
// Placeholder values - measure your green line
#define GREEN_RED_MIN 50
#define GREEN_RED_MAX 70
#define GREEN_GREEN_MIN 30
#define GREEN_GREEN_MAX 50
#define GREEN_BLUE_MIN 40
#define GREEN_BLUE_MAX 60

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

// Robot state machine
// STATE_FOLLOW_LINE: Normal line following (black/red/white)
// STATE_BLUE_DETECTED_1: First blue detected, freeze then 180
// STATE_FOLLOW_TO_BLACK: Following line until black detected
// STATE_ROTATE_TO_GREEN: Rotating until green detected
// STATE_FOLLOW_GREEN: Following green line
// STATE_BLUE_DETECTED_2: Second blue detected, freeze then 180
// STATE_RETURN_PATH: Following green then black back
// STATE_DONE: Finished
enum RobotState {
  STATE_FOLLOW_LINE,
  STATE_BLUE_DETECTED_1,
  STATE_FOLLOW_TO_BLACK,
  STATE_ROTATE_TO_GREEN,
  STATE_FOLLOW_GREEN,
  STATE_BLUE_DETECTED_2,
  STATE_RETURN_PATH,
  STATE_DONE
};

RobotState currentState = STATE_FOLLOW_LINE;
int blueDetectCount = 0;  // Tracks how many times blue has been detected

// Color confirmation counters (require multiple readings to confirm colors)
#define COLOR_CONFIRM_COUNT 5    // Number of consecutive readings required
int blueConfirmCount = 0;
int blackConfirmCount = 0;
int greenConfirmCount = 0;

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
  // Read color sensor
  readColorFast();
  
  switch (currentState) {
    
    case STATE_FOLLOW_LINE:
      // Normal line following - check for blue
      if (isBlue()) {
        blueConfirmCount++;
        if (blueConfirmCount >= COLOR_CONFIRM_COUNT) {
          Serial.println("BLUE detected - freezing for 1 second");
          stopMotors();
          delay(1000);  // Freeze for 1 second
          
          Serial.println("Fast 180 counterclockwise");
          turn180FastCCW();
          turn180FastCCW();
          turn180FastCCW();
          
          blueConfirmCount = 0;
          blueDetectCount++;
          currentState = STATE_FOLLOW_TO_BLACK;
          Serial.println("Now following line until BLACK detected");
          return;
        }
      } else {
        blueConfirmCount = 0;
      }
      
      // Do line following
      doLineFollowing();
      break;
      
    case STATE_FOLLOW_TO_BLACK:
      // Follow line until we detect black
      if (isBlack()) {
        blackConfirmCount++;
        if (blackConfirmCount >= COLOR_CONFIRM_COUNT) {
          Serial.println("BLACK detected - stopping and rotating to find GREEN");
          stopMotors();
          delay(200);
          blackConfirmCount = 0;
          currentState = STATE_ROTATE_TO_GREEN;
          return;
        }
      } else {
        blackConfirmCount = 0;
      }
      
      // Continue line following
      doLineFollowing();
      break;
      
    case STATE_ROTATE_TO_GREEN:
      // Rotate in place until green is detected
      rotateSlowRight();  // Slow rotation to find green
      
      if (isGreen()) {
        greenConfirmCount++;
        if (greenConfirmCount >= COLOR_CONFIRM_COUNT) {
          Serial.println("GREEN detected - following green line");
          stopMotors();
          delay(100);
          greenConfirmCount = 0;
          currentState = STATE_FOLLOW_GREEN;
          return;
        }
      } else {
        greenConfirmCount = 0;
      }
      break;
      
    case STATE_FOLLOW_GREEN:
      // Follow green line until blue detected
      if (isBlue()) {
        blueConfirmCount++;
        if (blueConfirmCount >= COLOR_CONFIRM_COUNT) {
          Serial.println("BLUE detected again - freezing for 1 second");
          stopMotors();
          delay(1000);  // Freeze for 1 second
          
          Serial.println("Slow 180 counterclockwise");
          turn180SlowCCW();
          turn180SlowCCW();
          turn180SlowCCW();
          
          blueConfirmCount = 0;
          blueDetectCount++;
          currentState = STATE_RETURN_PATH;
          Serial.println("Returning on green then black path");
          return;
        }
      } else {
        blueConfirmCount = 0;
      }
      
      // Follow green line (use green channel for error)
      doLineFollowingGreen();
      break;
      
    case STATE_RETURN_PATH:
      // Follow green path back, then black path
      // Just do general line following - will follow whatever line it's on
      doLineFollowing();
      break;
      
    case STATE_DONE:
      stopMotors();
      delay(100);
      break;
  }
  
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

// Turn 180 degrees FAST counterclockwise (left)
#define TURN_SPEED_FAST 200
void turn180FastCCW() {
  // Left motor backward, right motor forward (counterclockwise)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, TURN_SPEED_FAST);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, TURN_SPEED_FAST);
  
  delay(TURN_TIME_90 * 2);  // Double the time for 180
  stopMotors();
  delay(100);
}

// Turn 180 degrees SLOW counterclockwise (left)
#define TURN_SPEED_SLOW 100
void turn180SlowCCW() {
  // Left motor backward, right motor forward (counterclockwise)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, TURN_SPEED_SLOW);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, TURN_SPEED_SLOW);
  
  delay(TURN_TIME_90 * 3);  // Slower speed needs more time
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

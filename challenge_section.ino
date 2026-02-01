// challenge_section.ino
// Arduino sketch: follow black line, split->take right (green), PID follow green,
// detect blue, turn right 90, pick up box, turn 180, place box on blue square, turn 180,
// when splits choose right path, continue with PID follow until next reupload.
// RTOS-based for improved real-time line following.

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>

// -- PIN CONFIG (updated) --
// Color sensor (TCS230/TCS3200 style) pins - ONLY sensor for line following
const int COLOR_OUT = A1; // OUT pin from color sensor
const int S0_PIN = 2;
const int S1_PIN = 3;
const int S2_PIN = 4;
const int S3_PIN = 7;

// Motor driver pins (new mapping)
const int M1_PWM = 11; // ENA (left PWM)
const int M2_PWM = 10; // ENB (right PWM)
const int M1_DIR = 13; // IN1
const int M1_DIR2 = 12; // IN2
const int M2_DIR = 9;  // IN3
const int M2_DIR2 = 8; // IN4

const int SERVO_PIN = 6; // servo now on pin 6
Servo gripper;

// -- PID constants (tune these) --
float Kp = 0.9;
float Ki = 0.0;
float Kd = 0.12;

// -- Driving parameters --
int baseSpeed = 130; // base PWM speed (0-255)
int maxSpeed = 220;

// -- Timing constants (tune experimentally) --
const unsigned long TURN_90_MS = 400;  // time to rotate ~90 degrees in-place
const unsigned long TURN_180_MS = 800; // time to rotate ~180 degrees in-place
const int APPROACH_MS = 600; // forward approach time when picking up box

// Color calibration (frequency values - lower = stronger):
// Green: R:78 G:37 B:60 | White: R:14 G:15 B:12 | Red: R:20 G:90 B:67 | Black: R:140 G:150 B:129
int blackThreshold = 100;   // all channels above this = black line
int whiteThreshold = 40;    // all channels below this = white background

// PID state
volatile float lastError = 0;
volatile float integral = 0;
volatile unsigned long lastTime = 0;
volatile int lastDirection = 1; // +1 = drift right, -1 = drift left

// Shared sensor data (protected by mutex)
volatile unsigned long colorR = 0, colorG = 0, colorB = 0;
volatile float lineError = 0;
volatile bool blueDetected = false;
volatile bool greenDetected = false;
volatile bool redDetected = false;
volatile bool onBlackLine = false;
volatile bool lineValid = true;

// Mission state
enum MissionState { FOLLOW_BLACK, TAKE_RIGHT, FOLLOW_GREEN, BLUE_PICKUP, DONE };
volatile MissionState missionState = FOLLOW_BLACK;

// FreeRTOS handles
SemaphoreHandle_t sensorMutex;
TaskHandle_t sensorTaskHandle;
TaskHandle_t pidTaskHandle;
TaskHandle_t missionTaskHandle;

// helper: set motor speeds (positive = forward, negative = backward)
// leftSpeed/rightSpeed positive = forward
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor (use two direction pins)
  if (leftSpeed >= 0) {
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M1_DIR2, LOW);
    analogWrite(M1_PWM, constrain(leftSpeed, 0, maxSpeed));
  } else {
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M1_DIR2, HIGH);
    analogWrite(M1_PWM, constrain(-leftSpeed, 0, maxSpeed));
  }
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(M2_DIR, HIGH);
    digitalWrite(M2_DIR2, LOW);
    analogWrite(M2_PWM, constrain(rightSpeed, 0, maxSpeed));
  } else {
    digitalWrite(M2_DIR, LOW);
    digitalWrite(M2_DIR2, HIGH);
    analogWrite(M2_PWM, constrain(-rightSpeed, 0, maxSpeed));
  }
}

// Line following using color sensor only
bool isOnBlack(unsigned long r, unsigned long g, unsigned long b) {
  return (r > blackThreshold && g > blackThreshold && b > blackThreshold);
}

bool isOnWhite(unsigned long r, unsigned long g, unsigned long b) {
  return (r < whiteThreshold && g < whiteThreshold && b < whiteThreshold);
}

// PID controller returns steering correction (-255..+255)
int pidCompute(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (lastTime == 0) dt = 0.01;
  lastTime = now;

  if (error == 1000) {
    // lost line: small backup or last known steering
    integral = 0;
    return 0;
  }

  float e = error; // error in -2..2
  integral += e * dt;
  float derivative = (e - lastError) / max(dt, 0.0001);
  lastError = e;
  float output = Kp * e + Ki * integral + Kd * derivative; // output in roughly -?
  // scale output to motor offset
  int steer = (int)constrain(output * 60.0, -200, 200); // tuning multiplier
  return steer;
}

// Detect colors based on frequency values (lower = stronger color)
bool isGreen(unsigned long r, unsigned long g, unsigned long b) {
  // Green has lowest G value (~37), with R:78, B:60
  return (g < r && g < b && g < 50 && r > 60 && b > 45);
}

bool isRed(unsigned long r, unsigned long g, unsigned long b) {
  // Red has lowest R value (~20), with G:90, B:67
  return (r < g && r < b && r < 35 && g > 70 && b > 50);
}

bool checkBlue(unsigned long r, unsigned long g, unsigned long b) {
  // Blue has lowest B value (strongest blue reflection)
  return (b < r && b < g && b < 50 && r > 60 && g > 50);
}

void turnInPlace(int direction, unsigned long durationMs) {
  // direction: +1 = right, -1 = left (rotate in place)
  int turnSpeed = 150;
  if (direction > 0) {
    // right: left forward, right backward
    setMotors(turnSpeed, -turnSpeed);
  } else {
    setMotors(-turnSpeed, turnSpeed);
  }
  delay(durationMs);
  setMotors(0, 0);
}

void turnRight90() { turnInPlace(+1, TURN_90_MS); }
void turn180() { turnInPlace(+1, TURN_180_MS); } // same direction twice duration

void pickupBox() {
  // approach a bit, close gripper
  setMotors(baseSpeed / 2, baseSpeed / 2);
  delay(APPROACH_MS);
  setMotors(0,0);
  gripper.write(11); // close to pick up box
  delay(500);
}

void placeBox() {
  gripper.write(0); // open position
  delay(500);
}

// ===== RTOS TASKS =====

// Color sensor read function
unsigned long readColorFreqTask(int s2, int s3) {
  digitalWrite(S2_PIN, s2);
  digitalWrite(S3_PIN, s3);
  delayMicroseconds(100);
  unsigned long duration = pulseIn(COLOR_OUT, LOW, 30000);
  if (duration == 0) return 30000;
  return duration;
}

// High-priority: read color sensor for line following at ~100Hz
void TaskSensorRead(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Read RGB values
    unsigned long r = readColorFreqTask(LOW, LOW);
    unsigned long g = readColorFreqTask(HIGH, HIGH);
    unsigned long b = readColorFreqTask(LOW, HIGH);
    
    // Determine line state
    bool onBlack = isOnBlack(r, g, b);
    bool onWhite = isOnWhite(r, g, b);
    bool green = isGreen(r, g, b);
    bool red = isRed(r, g, b);
    bool blue = checkBlue(r, g, b);
    
    // Calculate line error for PID
    float error = 0;
    bool valid = true;
    if (onBlack || green) {
      // On line - no error
      error = 0;
      valid = true;
    } else if (onWhite) {
      // Lost line
      error = 1000;
      valid = false;
    } else {
      // Transitioning - calculate proportional error
      float avg = (r + g + b) / 3.0;
      float normalized = (blackThreshold - avg) / (float)(blackThreshold - whiteThreshold);
      normalized = constrain(normalized, 0, 1);
      error = normalized * lastDirection;
      valid = true;
    }
    
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      colorR = r; colorG = g; colorB = b;
      lineError = error;
      lineValid = valid;
      onBlackLine = onBlack;
      blueDetected = blue;
      greenDetected = green;
      redDetected = red;
      xSemaphoreGive(sensorMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz
  }
}

// Medium-priority: PID motor control at ~100Hz
void TaskPIDControl(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    float err;
    bool valid;
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      err = lineError;
      valid = lineValid;
      xSemaphoreGive(sensorMutex);
    }
    if (missionState == FOLLOW_BLACK || missionState == TAKE_RIGHT || missionState == FOLLOW_GREEN) {
      if (!valid || err == 1000) {
        // Lost line - search in last known direction
        int searchSpeed = 70;
        if (lastDirection > 0) {
          setMotors(searchSpeed, -searchSpeed/2);
        } else {
          setMotors(-searchSpeed/2, searchSpeed);
        }
      } else {
        // Update last direction
        if (err > 0.1) lastDirection = 1;
        else if (err < -0.1) lastDirection = -1;
        
        int steer = pidCompute(err);
        int left = baseSpeed - steer;
        int right = baseSpeed + steer;
        setMotors(left, right);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz
  }
}

// Color detection is now integrated into TaskSensorRead

// Mission state machine task
void TaskMission(void *pvParameters) {
  (void) pvParameters;
  bool localBlue, localGreen, localRed;
  
  // State: FOLLOW_BLACK until we see green or red (split)
  missionState = FOLLOW_BLACK;
  Serial.println("Starting: following black line");
  for (;;) {
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      localBlue = blueDetected;
      localGreen = greenDetected;
      localRed = redDetected;
      xSemaphoreGive(sensorMutex);
    }
    
    if (missionState == FOLLOW_BLACK) {
      if (localBlue) {
        missionState = BLUE_PICKUP;
        setMotors(0,0);
        turnRight90(); pickupBox(); turn180();
        setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(800)); setMotors(0,0);
        placeBox(); turn180();
        missionState = FOLLOW_BLACK;
      } else if (localGreen || localRed) {
        // Split detected - take right (green)
        Serial.println("Split detected: taking right (green)");
        missionState = TAKE_RIGHT;
        setMotors(0,0);
        turnInPlace(+1, 150); // Turn right toward green
        setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(300)); setMotors(0,0);
        missionState = FOLLOW_GREEN;
      }
    } else if (missionState == FOLLOW_GREEN) {
      if (localBlue) {
        missionState = BLUE_PICKUP;
        setMotors(0,0);
        Serial.println("BLUE on green: pickup/drop");
        turnRight90(); pickupBox(); turn180();
        setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(800)); setMotors(0,0);
        placeBox(); turn180();
        missionState = FOLLOW_GREEN;
      } else if (localRed) {
        // Another split - choose right
        Serial.println("Next split: choosing right");
        setMotors(0,0);
        turnInPlace(+1, 150);
        setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(300)); setMotors(0,0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  // Color sensor pins
  pinMode(COLOR_OUT, INPUT);
  pinMode(S0_PIN, OUTPUT); pinMode(S1_PIN, OUTPUT); pinMode(S2_PIN, OUTPUT); pinMode(S3_PIN, OUTPUT);
  digitalWrite(S0_PIN, HIGH); digitalWrite(S1_PIN, LOW); // 20% frequency scaling

  // Motor pins
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT); pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT); pinMode(M2_DIR2, OUTPUT);
  
  // Servo
  gripper.attach(SERVO_PIN);
  gripper.write(0); // default open position

  Serial.begin(115200);
  lastTime = millis();

  // Create mutex
  sensorMutex = xSemaphoreCreateMutex();

  // Create tasks with priorities (higher number = higher priority)
  xTaskCreate(TaskSensorRead,  "Sensors", 192, NULL, 3, &sensorTaskHandle);
  xTaskCreate(TaskPIDControl,  "PID",     128, NULL, 2, &pidTaskHandle);
  xTaskCreate(TaskMission,     "Mission", 256, NULL, 1, &missionTaskHandle);

  // Start scheduler (never returns)
  vTaskStartScheduler();
}

// With RTOS, the loop() is empty; all work is done in tasks.
void loop() {
  // FreeRTOS scheduler runs tasks; loop() should not execute.
  // If we reach here, scheduler failed.
  Serial.println("ERROR: RTOS scheduler not running");
  while(1) delay(1000);
}

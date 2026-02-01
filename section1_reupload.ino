// section1_reupload.ino
// Arduino sketch for Section 1 (after first reupload) - RTOS version
// Behavior: follow black line -> when it splits take right (green) -> PID follow green
// detect blue -> turn right 90, pick up box -> turn 180, place box on blue square -> turn 180
// when splits choose right path -> follow (turning) using PID -> stop at reupload point

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>

// Pin configuration (updated)
const int COLOR_OUT = A1;
const int S0_PIN = 2; const int S1_PIN = 3; const int S2_PIN = 4; const int S3_PIN = 7;

const int M1_PWM = 11; const int M1_DIR = 13; const int M1_DIR2 = 12;
const int M2_PWM = 10; const int M2_DIR = 9;  const int M2_DIR2 = 8;
const int SERVO_PIN = 6;

Servo gripper;

// PID params
float Kp = 0.9; float Ki = 0.0; float Kd = 0.12;
int baseSpeed = 130; int maxSpeed = 220;

// timing (tune per hardware)
const unsigned long TURN_90_MS = 400;
const unsigned long TURN_180_MS = 800;
const int APPROACH_MS = 600;

// Color calibration: Black R:140 G:150 B:129 | White R:14 G:15 B:12 | Green R:78 G:37 B:60 | Red R:20 G:90 B:67
int blackThreshold = 100; int whiteThreshold = 40;
int greenRedMaxB = 70; // Blue channel threshold to distinguish green/red from black

// PID state (volatile for RTOS)
volatile float lastError = 0; 
volatile float integral = 0; 
volatile unsigned long lastTime = 0;
volatile int lastDrift = 0; // -1 = drifted left, +1 = drifted right (for search)

// Shared sensor data (protected by mutex)
volatile float lineError = 0;
volatile bool blueDetected = false;
volatile bool greenDetected = false;
volatile bool redDetected = false;
volatile bool lineValid = true;

// Mission state
enum MissionState { FOLLOW_BLACK, TAKE_RIGHT, FOLLOW_GREEN, BLUE_PICKUP, WAIT_REUPLOAD };
volatile MissionState missionState = FOLLOW_BLACK;
volatile int splitCount = 0;

// FreeRTOS handles
SemaphoreHandle_t sensorMutex;
TaskHandle_t sensorTaskHandle;
TaskHandle_t pidTaskHandle;
TaskHandle_t missionTaskHandle;

void setMotors(int left, int right) {
  if (left >= 0) { digitalWrite(M1_DIR, HIGH); digitalWrite(M1_DIR2, LOW); analogWrite(M1_PWM, constrain(left,0,maxSpeed)); }
  else { digitalWrite(M1_DIR, LOW); digitalWrite(M1_DIR2, HIGH); analogWrite(M1_PWM, constrain(-left,0,maxSpeed)); }
  if (right >= 0) { digitalWrite(M2_DIR, HIGH); digitalWrite(M2_DIR2, LOW); analogWrite(M2_PWM, constrain(right,0,maxSpeed)); }
  else { digitalWrite(M2_DIR, LOW); digitalWrite(M2_DIR2, HIGH); analogWrite(M2_PWM, constrain(-right,0,maxSpeed)); }
}

int pidCompute(float error) {
  unsigned long now = millis(); float dt = (now - lastTime)/1000.0; if(lastTime==0) dt = 0.01; lastTime = now;
  if(error==1000){ integral=0; return 0; }
  float e = error; integral += e*dt; float derivative = (e-lastError)/max(dt,0.0001f); lastError = e;
  float out = Kp*e + Ki*integral + Kd*derivative; int steer = (int)constrain(out*60.0, -200, 200); return steer;
}

unsigned long readColorFreq(int s2, int s3) {
  digitalWrite(S2_PIN, s2);
  digitalWrite(S3_PIN, s3);
  vTaskDelay(pdMS_TO_TICKS(10));
  unsigned long duration = pulseIn(COLOR_OUT, LOW, 30000);
  if (duration == 0) return 30000;
  return duration;
}

bool detectBlue() {
  unsigned long r = readColorFreq(LOW, LOW);
  unsigned long g = readColorFreq(HIGH, HIGH);
  unsigned long b = readColorFreq(LOW, HIGH);
  // Blue: B is lowest (strongest), with R and G higher
  return (b < r && b < g && b < 50 && r > 60 && g > 50);
}

bool isOnBlack(unsigned long r, unsigned long g, unsigned long b) {
  return (r > blackThreshold && g > blackThreshold && b > blackThreshold);
}

bool isOnWhite(unsigned long r, unsigned long g, unsigned long b) {
  return (r < whiteThreshold && g < whiteThreshold && b < whiteThreshold);
}

bool detectGreen(unsigned long r, unsigned long g, unsigned long b) {
  // Green: G is low (strong green), R medium, B medium
  return (g < 50 && g < r && g < b && r > 60 && b < greenRedMaxB && b > 40);
}

bool detectRed(unsigned long r, unsigned long g, unsigned long b) {
  // Red: R is low (strong red), G high (weak), B medium
  return (r < 40 && g > 70 && b > 50 && b < 80);
}

void turnInPlace(int dir, unsigned long ms){ int t=150; if(dir>0) setMotors(t,-t); else setMotors(-t,t); vTaskDelay(pdMS_TO_TICKS(ms)); setMotors(0,0); }
void turnRight90(){ turnInPlace(+1, TURN_90_MS); }
void turn180(){ turnInPlace(+1, TURN_180_MS); }

void pickupBox(){ setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(APPROACH_MS)); setMotors(0,0); gripper.write(11); vTaskDelay(pdMS_TO_TICKS(400)); } // close to 11 for pickup
void placeBox(){ gripper.write(0); vTaskDelay(pdMS_TO_TICKS(400)); } // open to 0

// ===== RTOS TASKS =====

// High-priority: read color sensor for line following at ~50Hz
void TaskSensorRead(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static float prevR = 0, prevG = 0, prevB = 0;
  
  for (;;) {
    unsigned long r = readColorFreq(LOW, LOW);   // Red
    unsigned long g = readColorFreq(HIGH, HIGH); // Green
    unsigned long b = readColorFreq(LOW, HIGH);  // Blue
    
    float err = 0;
    bool valid = true;
    bool green = false, red = false, blue = false;
    
    // Check for special colors first
    if (detectBlue()) { blue = true; }
    else if (detectGreen(r, g, b)) { green = true; }
    else if (detectRed(r, g, b)) { red = true; }
    else if (isOnBlack(r, g, b)) {
      // On black line - centered
      err = 0;
      valid = true;
    } else if (isOnWhite(r, g, b)) {
      // Lost line - use last drift direction
      err = lastDrift * 2.0;
      valid = false;
    } else {
      // Transitioning - calculate proportional error
      float avgNow = (r + g + b) / 3.0;
      float avgPrev = (prevR + prevG + prevB) / 3.0;
      if (avgNow > avgPrev) {
        // Moving toward white - we're drifting off
        err = (avgNow - blackThreshold) / 50.0;
        if (lastDrift != 0) err *= lastDrift;
      } else {
        // Moving toward black - good
        err = 0;
      }
      valid = true;
    }
    
    // Update drift direction based on motor commands later
    prevR = r; prevG = g; prevB = b;
    
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      lineError = err;
      lineValid = valid;
      blueDetected = blue;
      greenDetected = green;
      redDetected = red;
      xSemaphoreGive(sensorMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz
  }
}

// Medium-priority: PID motor control at ~100Hz
void TaskPIDControl(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static unsigned long lostTime = 0;
  
  for (;;) {
    float err;
    bool valid;
    MissionState state = missionState;
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      err = lineError;
      valid = lineValid;
      xSemaphoreGive(sensorMutex);
    }
    if (state == FOLLOW_BLACK || state == TAKE_RIGHT || state == FOLLOW_GREEN) {
      if (!valid) {
        // Lost line - search behavior
        if (lostTime == 0) lostTime = millis();
        unsigned long lost = millis() - lostTime;
        int searchSpeed = 100;
        if (lost < 300) {
          // Continue in last drift direction
          if (lastDrift >= 0) setMotors(searchSpeed, searchSpeed/3);
          else setMotors(searchSpeed/3, searchSpeed);
        } else if (lost < 800) {
          // Wider search - turn in place
          if (lastDrift >= 0) setMotors(searchSpeed, -searchSpeed/2);
          else setMotors(-searchSpeed/2, searchSpeed);
        } else {
          // Very lost - slow forward
          setMotors(baseSpeed/3, baseSpeed/3);
        }
      } else {
        lostTime = 0;
        int steer = pidCompute(err);
        int left = baseSpeed - steer;
        int right = baseSpeed + steer;
        setMotors(left, right);
        // Update drift direction
        if (steer > 10) lastDrift = 1;  // Steering right = drifted right
        else if (steer < -10) lastDrift = -1;
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
  
  missionState = FOLLOW_BLACK;
  Serial.println("Section1: following black line");
  
  for (;;) {
    if (missionState == WAIT_REUPLOAD) {
      setMotors(0,0);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
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
        Serial.println("Split: taking right (green)");
        splitCount++;
        missionState = TAKE_RIGHT;
        setMotors(0,0);
        turnInPlace(+1, 150);
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
      } else if (localGreen || localRed) {
        splitCount++;
        Serial.println("Next split: choosing right");
        setMotors(0,0);
        turnInPlace(+1, 150);
        setMotors(baseSpeed/2, baseSpeed/2); vTaskDelay(pdMS_TO_TICKS(300)); setMotors(0,0);
        // After second split, stop at reupload point
        if (splitCount >= 2) {
          Serial.println("Reached reupload point. Stopping.");
          missionState = WAIT_REUPLOAD;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup(){
  // Color sensor pins
  pinMode(COLOR_OUT, INPUT);
  pinMode(S0_PIN, OUTPUT); pinMode(S1_PIN, OUTPUT); pinMode(S2_PIN, OUTPUT); pinMode(S3_PIN, OUTPUT);
  digitalWrite(S0_PIN, HIGH); digitalWrite(S1_PIN, LOW); // 20% frequency scaling
  
  // Motor pins
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT); pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT); pinMode(M2_DIR2, OUTPUT);
  
  // Servo
  gripper.attach(SERVO_PIN); gripper.write(0); // default open
  
  Serial.begin(115200); lastTime = millis();

  // Create mutex
  sensorMutex = xSemaphoreCreateMutex();

  // Create tasks with priorities (higher = more important)
  xTaskCreate(TaskSensorRead,  "Sensors", 192, NULL, 3, &sensorTaskHandle);
  xTaskCreate(TaskPIDControl,  "PID",     128, NULL, 2, &pidTaskHandle);
  xTaskCreate(TaskMission,     "Mission", 256, NULL, 1, &missionTaskHandle);

  // Start scheduler
  vTaskStartScheduler();
}

void loop(){
  // RTOS handles everything; loop should not run
  Serial.println("ERROR: RTOS scheduler not running");
  while(1) delay(1000);
}

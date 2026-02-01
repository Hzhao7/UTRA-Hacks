// test_line_follow.ino
// Simple test sketch to verify black line following using COLOR SENSOR ONLY
// Uses same pin config as main sketches - no RTOS for easier debugging

// ----- PIN CONFIG -----
// Color sensor (TCS230/TCS3200 style) pins
const int COLOR_OUT = A1; // OUT pin from color sensor
const int S0_PIN = 2;
const int S1_PIN = 3;
const int S2_PIN = 4;
const int S3_PIN = 7;

const int M1_PWM = 11; // ENA (left motor)
const int M1_DIR = 13; // IN1
const int M1_DIR2 = 12; // IN2
const int M2_PWM = 10; // ENB (right motor)
const int M2_DIR = 9;  // IN3
const int M2_DIR2 = 8; // IN4

// ----- PID params (tune these) -----
float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.15;

int baseSpeed = 80; // start slow for testing
int maxSpeed = 180;

// ----- Color Calibration -----
// Black: R:140 G:150 B:129 (high values = weak reflection)
// White: R:14 G:15 B:12 (low values = strong reflection)
int blackThreshold = 100;  // all channels above this = on black line
int whiteThreshold = 40;   // all channels below this = on white

// ----- PID state -----
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;
int lastDirection = 1; // remember last drift direction (+1 right, -1 left)

// Color readings
unsigned long colorR = 0, colorG = 0, colorB = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Line Follow Test (Color Sensor) ===");
  
  // Setup color sensor pins
  pinMode(COLOR_OUT, INPUT);
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  
  // Set frequency scaling to 20% (S0=HIGH, S1=LOW)
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);
  
  // Setup motor pins
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);
  
  // Stop motors initially
  setMotors(0, 0);
  
  Serial.println("Place robot on black line, then open Serial Monitor");
  Serial.println("Commands: 's'=start, 'x'=stop, 'c'=read colors");
  Serial.println("'+'/'-'=speed, 'p'/'P'=Kp, 'd'/'D'=Kd");
  
  lastTime = millis();

  // Auto-start so the robot begins following immediately after upload
  running = true;
  Serial.println("AUTO-START ENABLED - following line");
}

void setMotors(int left, int right) {
  // Left motor
  if (left >= 0) {
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M1_DIR2, LOW);
    analogWrite(M1_PWM, constrain(left, 0, maxSpeed));
  } else {
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M1_DIR2, HIGH);
    analogWrite(M1_PWM, constrain(-left, 0, maxSpeed));
  }
  // Right motor
  if (right >= 0) {
    digitalWrite(M2_DIR, HIGH);
    digitalWrite(M2_DIR2, LOW);
    analogWrite(M2_PWM, constrain(right, 0, maxSpeed));
  } else {
    digitalWrite(M2_DIR, LOW);
    digitalWrite(M2_DIR2, HIGH);
    analogWrite(M2_PWM, constrain(-right, 0, maxSpeed));
  }
}

unsigned long readColorFreq(int s2, int s3) {
  digitalWrite(S2_PIN, s2);
  digitalWrite(S3_PIN, s3);
  delayMicroseconds(100);
  unsigned long duration = pulseIn(COLOR_OUT, LOW, 30000);
  if (duration == 0) return 30000;
  return duration;
}

void readColors() {
  colorR = readColorFreq(LOW, LOW);   // Red
  colorG = readColorFreq(HIGH, HIGH); // Green  
  colorB = readColorFreq(LOW, HIGH);  // Blue
}

bool isOnBlack() {
  // Black has HIGH frequency values (weak reflection)
  return (colorR > blackThreshold && colorG > blackThreshold && colorB > blackThreshold);
}

bool isOnWhite() {
  // White has LOW frequency values (strong reflection)
  return (colorR < whiteThreshold && colorG < whiteThreshold && colorB < whiteThreshold);
}

// Returns line error: 0 = centered on black, positive = drifting, 1000 = line lost
float readLineError() {
  readColors();
  
  if (isOnBlack()) {
    // On the line - no error
    return 0;
  } else if (isOnWhite()) {
    // Lost the line completely - return large error in last known direction
    return 1000;
  } else {
    // Transitioning - calculate proportional error based on how "white" we're getting
    // Average of RGB values - lower means more white (off line)
    float avg = (colorR + colorG + colorB) / 3.0;
    // Map from blackThreshold to whiteThreshold -> 0 to 1
    float normalized = (blackThreshold - avg) / (float)(blackThreshold - whiteThreshold);
    normalized = constrain(normalized, 0, 1);
    // Return signed error based on last direction
    return normalized * lastDirection;
  }
}

int pidCompute(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (lastTime == 0 || dt < 0.001) dt = 0.01;
  lastTime = now;
  
  if (error == 1000) {
    // Line lost - return steering in last known direction
    integral = 0;
    return lastDirection * 80; // Turn to search
  }
  
  // Update last direction based on current error
  if (error > 0.1) lastDirection = 1;
  else if (error < -0.1) lastDirection = -1;
  
  integral += error * dt;
  integral = constrain(integral, -10, 10); // Anti-windup
  float derivative = (error - lastError) / dt;
  lastError = error;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  return (int)constrain(output * 100.0, -150, 150);
}

void printColors() {
  Serial.print("R:"); Serial.print(colorR);
  Serial.print(" G:"); Serial.print(colorG);
  Serial.print(" B:"); Serial.print(colorB);
  Serial.print(" | ");
  if (isOnBlack()) Serial.print("BLACK (on line)");
  else if (isOnWhite()) Serial.print("WHITE (off line)");
  else Serial.print("EDGE");
  Serial.println();
}

bool running = false;

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      running = true;
      Serial.println("STARTED - following line");
    } else if (cmd == 'x' || cmd == 'X') {
      running = false;
      setMotors(0, 0);
      Serial.println("STOPPED");
    } else if (cmd == 'c' || cmd == 'C') {
      readColors();
      printColors();
    } else if (cmd == '+') {
      baseSpeed += 10;
      Serial.print("Speed: "); Serial.println(baseSpeed);
    } else if (cmd == '-') {
      baseSpeed -= 10;
      Serial.print("Speed: "); Serial.println(baseSpeed);
    } else if (cmd == 'p') {
      Kp += 0.1;
      Serial.print("Kp: "); Serial.println(Kp);
    } else if (cmd == 'P') {
      Kp -= 0.1;
      Serial.print("Kp: "); Serial.println(Kp);
    } else if (cmd == 'd') {
      Kd += 0.02;
      Serial.print("Kd: "); Serial.println(Kd);
    } else if (cmd == 'D') {
      Kd -= 0.02;
      Serial.print("Kd: "); Serial.println(Kd);
    }
  }
  
  if (running) {
    float err = readLineError();
    
    if (err == 1000) {
      // Lost line - turn to search in last known direction
      int searchSpeed = 60;
      if (lastDirection > 0) {
        setMotors(searchSpeed, -searchSpeed/2); // Turn right
      } else {
        setMotors(-searchSpeed/2, searchSpeed); // Turn left
      }
      Serial.println("Line lost - searching...");
    } else {
      int steer = pidCompute(err);
      int left = baseSpeed - steer;
      int right = baseSpeed + steer;
      setMotors(left, right);
    }
    
    // Print status every 300ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 300) {
      printColors();
      lastPrint = millis();
    }
  }
  
  delay(15); // ~66Hz loop rate
}

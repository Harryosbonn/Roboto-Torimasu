#include <Servo.h>
#include <Wire.h>

// --- KONFIGURATION ---
#define PIN_SERVO 9
#define PIN_ESC 10
#define PIN_BUZZER 11
#define PIN_TRIG 12
#define PIN_ECHO 13

// MPU6050 I2C Address
const int MPU_ADDR = 0x68;

// Safety Thresholds
const int MAX_TILT_ANGLE = 60; // Degrees (Updated from 45)
const int OBSTACLE_DIST_CM = 15; // Emergency Stop Distance (Updated from 30)
const unsigned long SAFETY_TIMEOUT_MS = 1000; // Stop if no command for 1s
const unsigned long TIP_DEBOUNCE_MS = 500;    // Persistence required for alarm

// Globals
Servo steeringServo;
Servo throttleESC; // Most ESCs accept Servo logic
unsigned long lastCommandTime = 0;
unsigned long firstTippedTime = 0; // For debounce
float pitch = 0;
float roll = 0;
bool alarmActive = false;

struct Command {
  int steer;    // 0-180 (90 center)
  int throttle; // 1000-2000 (1500 neutral)
};

Command currentCmd = {90, 1500}; // Default Neutral

void setup() {
  Serial.begin(115200);   // Command Serial (USB)
  Serial.setTimeout(5);   // Low timeout for non-blocking feel
  
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Init Motors
  steeringServo.attach(PIN_SERVO);
  throttleESC.attach(PIN_ESC); 
  
  // Arm ESC (Send neutral for a few seconds)
  throttleESC.writeMicroseconds(1500); 
  steeringServo.write(90);
  delay(2000);

  // Init MPU6050 (Bypassed if hang suspected)
  Serial.println("INIT: Initializing I2C...");
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Try to read instead of write to wake
  if (Wire.endTransmission() == 0) {
    Serial.println("INIT: IMU Found. Waking up...");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);     
    Wire.endTransmission(true);
  } else {
    Serial.println("INIT: IMU NOT FOUND! Bypassing...");
  }
  
  Serial.println("READY");
}

void loop() {
  // 1. Read Sensors
  readIMU();
  long dist = readUltrasonic();

  // 2. Safety Checks
  bool isTipped = (abs(pitch) > MAX_TILT_ANGLE || abs(roll) > MAX_TILT_ANGLE);
  bool isBlocked = (dist > 0 && dist < OBSTACLE_DIST_CM);
  bool isTimedOut = (millis() - lastCommandTime) > SAFETY_TIMEOUT_MS;

  if (isTipped) {
    if (firstTippedTime == 0) firstTippedTime = millis();
    
    if (millis() - firstTippedTime > TIP_DEBOUNCE_MS) {
      alarmActive = true;
      stopMotors();
      digitalWrite(PIN_BUZZER, HIGH); // Constant tone
    }
  } else {
    firstTippedTime = 0;
    alarmActive = false;
    digitalWrite(PIN_BUZZER, LOW);
  }

  // 3. Process Serial Commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      int s = Serial.parseInt();
      int t = Serial.parseInt();
      if (Serial.read() == '>') {
        currentCmd.steer = constrain(s, 0, 180);
        currentCmd.throttle = constrain(t, 1000, 2000); 
        lastCommandTime = millis();
      }
    }
  }

  // 4. Actuate
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 200) { 
    Serial.print("DEBUG | Pitch:"); Serial.print(pitch);
    Serial.print("|Roll:"); Serial.print(roll);
    Serial.print("|Dist:"); Serial.print(dist);
    Serial.print("|Tipped:"); Serial.print(isTipped);
    Serial.print("|Blocked:"); Serial.print(isBlocked);
    Serial.print("|Timeout:"); Serial.print(isTimedOut);
    Serial.print("|Steer:"); Serial.print(currentCmd.steer);
    Serial.print("|Throttle:"); Serial.println(currentCmd.throttle);
    lastDebugTime = millis();
  }

  if (!alarmActive && !isTimedOut) {
    // Obstacle Override
    if (isBlocked && currentCmd.throttle > 1500) { 
       throttleESC.writeMicroseconds(1500);
       tone(PIN_BUZZER, 1000, 100); // Beep warning
    } else {
       steeringServo.write(currentCmd.steer);
       throttleESC.writeMicroseconds(currentCmd.throttle);
    }
  } else {
    stopMotors();
  }

  delay(5); 
}

void stopMotors() {
  steeringServo.write(90);
  throttleESC.writeMicroseconds(1500);
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() >= 6) {
    int16_t AcX = Wire.read()<<8|Wire.read();
    int16_t AcY = Wire.read()<<8|Wire.read();
    int16_t AcZ = Wire.read()<<8|Wire.read();
    pitch = atan2(AcY, AcZ) * 180 / PI;
    roll  = atan2(-AcX, AcZ) * 180 / PI;
  }
}

long readUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

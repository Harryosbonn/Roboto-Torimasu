#include <Servo.h>
#include <Wire.h>
#include <avr/wdt.h>  // Hardware watchdog timer

// --- KONFIGURATION ---
#define PIN_SERVO 9
#define PIN_ENA 10
#define PIN_IN1 7
#define PIN_IN2 8
#define PIN_BUZZER 11
#define PIN_TRIG 12
#define PIN_ECHO 13

// MPU6050 I2C Address
const int MPU_ADDR = 0x68;

// Safety Thresholds
// Safety Thresholds
const int MAX_TILT_ANGLE = 45; // Degrees (Stricter)
const int OBSTACLE_STOP_CM = 15;  // Emergency Stop Distance
const int OBSTACLE_SLOW_CM = 40;  // Start slowing down
const unsigned long SAFETY_TIMEOUT_MS = 500; // Stop if no command for 0.5s
const unsigned long TIP_DEBOUNCE_MS = 200;    // Persistence required for alarm

// Ultrasonic averaging buffer
const int US_BUFFER_SIZE = 3;
long usBuffer[US_BUFFER_SIZE] = {999, 999, 999};
int usBufferIndex = 0;

// Globals
Servo steeringServo;
// L298N uses direct PWM and direction pins instead of Servo logic
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
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  
  // Init Motor Driver (L298N)
  Serial.println("INIT: Motor Driver Ready");
  stopMotors();
  delay(500);

  // Init MPU6050
  // Init MPU6050
  Serial.println("INIT: Initializing I2C...");
  Wire.begin();
  Wire.setWireTimeout(3000, true); // 3ms timeout, reset on timeout
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  if (Wire.endTransmission() == 0) {
    Serial.println("INIT: IMU Found. Waking up...");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);     
    Wire.endTransmission(true);
  } else {
    Serial.println("INIT: IMU NOT FOUND! Bypassing...");
  }
  
  // Enable hardware watchdog (2 second timeout)
  // If loop() hangs for >2s, Arduino auto-resets
  wdt_enable(WDTO_2S);
  
  Serial.println("READY");
}

void loop() {
  // Pet the watchdog - prevents auto-reset as long as loop runs
  wdt_reset();
  
  // 1. Read Sensors
  readIMU();
  long rawDist = readUltrasonic();
  
  // Update ultrasonic averaging buffer
  usBuffer[usBufferIndex] = rawDist;
  usBufferIndex = (usBufferIndex + 1) % US_BUFFER_SIZE;
  
  // Calculate averaged distance
  long dist = 0;
  for (int i = 0; i < US_BUFFER_SIZE; i++) {
    dist += usBuffer[i];
  }
  dist /= US_BUFFER_SIZE;

  // 2. Safety Checks
  bool isTipped = (abs(pitch) > MAX_TILT_ANGLE || abs(roll) > MAX_TILT_ANGLE);
  bool isBlocked = (dist > 5 && dist < OBSTACLE_STOP_CM);
  bool isSlowing = (dist >= OBSTACLE_STOP_CM && dist < OBSTACLE_SLOW_CM);
  bool isTimedOut = (millis() - lastCommandTime) > SAFETY_TIMEOUT_MS;

  if (isTipped) {
    if (firstTippedTime == 0) firstTippedTime = millis();
    
    if (millis() - firstTippedTime > TIP_DEBOUNCE_MS) {
      alarmActive = true;
      stopMotors();
      tone(PIN_BUZZER, 2000); // 2kHz tone for alarm
    }
  } else {
    firstTippedTime = 0;
    alarmActive = false;
    noTone(PIN_BUZZER);
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
  // 4. Actuate
  static int lastSteer = -1;
  static int lastThrottle = -1;
  static bool lastAlarmState = false;

  bool stateChanged = (alarmActive != lastAlarmState) || (isTimedOut != lastAlarmState); // excessive simplification, let's just force update on state change
  
  // Force update if safety state changes
  if (alarmActive || isTimedOut) {
      if (!lastAlarmState) { // Transition to safe state
         stopMotors();
         lastAlarmState = true;
         lastSteer = 90; 
         lastThrottle = 1500;
      }
      if (alarmActive) {
        tone(PIN_BUZZER, 2000);
      } else {
        noTone(PIN_BUZZER);
      }
  } else {
      lastAlarmState = false;
      
      // Determine target values
      int targetSteer = currentCmd.steer;
      int targetThrottle = currentCmd.throttle;

      // Obstacle Override - Emergency Stop
      if (isBlocked && targetThrottle > 1500) {
         targetThrottle = 1500;
         tone(PIN_BUZZER, 1000, 100);
      } 
      // Proportional slowdown in warning zone
      else if (isSlowing && targetThrottle > 1500) {
         // Scale throttle down: closer = slower
         // At OBSTACLE_SLOW_CM: full speed, at OBSTACLE_STOP_CM: zero
         float slowFactor = (float)(dist - OBSTACLE_STOP_CM) / (float)(OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM);
         slowFactor = constrain(slowFactor, 0.2, 1.0); // Min 20% speed
         int reduced = 1500 + (int)((targetThrottle - 1500) * slowFactor);
         targetThrottle = constrain(reduced, 1500, 2000);
         tone(PIN_BUZZER, 500, 50); // Soft warning beep
      } else {
         noTone(PIN_BUZZER);
      }

      // WRITE ONLY ON CHANGE (Reduces Jitter)
      if (targetSteer != lastSteer) {
          steeringServo.write(targetSteer);
          lastSteer = targetSteer;
      }

      if (targetThrottle != lastThrottle) {
          // Send throttle to L298N
          if (targetThrottle == 1500) {
              // Neutral / Stop
              digitalWrite(PIN_IN1, LOW);
              digitalWrite(PIN_IN2, LOW);
              analogWrite(PIN_ENA, 0);
          } else if (targetThrottle > 1500) {
              // Forward
              int speed = map(targetThrottle, 1500, 2000, 0, 255);
              digitalWrite(PIN_IN1, HIGH);
              digitalWrite(PIN_IN2, LOW);
              analogWrite(PIN_ENA, speed);
          } else {
              // Backward
              int speed = map(targetThrottle, 1500, 1000, 0, 255);
              digitalWrite(PIN_IN1, LOW);
              digitalWrite(PIN_IN2, HIGH);
              analogWrite(PIN_ENA, speed);
          }
          lastThrottle = targetThrottle;
      }
  }

  // Debug Output
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

  delay(2); // Reduced delay for responsiveness 
}

void stopMotors() {
  steeringServo.write(90);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);
}

void readIMU() {
  // Use timeout to prevent I2C hang when motors cause EMI
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    // I2C error - skip this read to prevent hang
    return;
  }
  
  // Request with timeout protection
  unsigned long i2cStart = millis();
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  // Wait for data with timeout (100ms max)
  while (Wire.available() < 6) {
    if (millis() - i2cStart > 100) {
      // Timeout - flush and return
      while (Wire.available()) Wire.read();
      return;
    }
  }
  
  int16_t AcX = Wire.read()<<8|Wire.read();
  int16_t AcY = Wire.read()<<8|Wire.read();
  int16_t AcZ = Wire.read()<<8|Wire.read();
  pitch = atan2(AcY, AcZ) * 180 / PI;
  roll  = atan2(-AcX, AcZ) * 180 / PI;
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

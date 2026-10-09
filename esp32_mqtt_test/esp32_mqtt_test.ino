#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

const char* ssid        = "ssid";
const char* password    = "pwd";
const char* mqtt_server = "ip";

#define TOPIC_COMMANDS "chair/commands"
#define TOPIC_STATUS   "chair/status"

// ============================================================
// SOUND SENSOR & MANUAL RESET SWITCH PINS
// ============================================================
#define SOUND_SENSOR_PIN 2
#define RESET_BUTTON_PIN 0  // GPIO 0 Push Button (BOOT Button or External Switch)

volatile bool clapDetectedFlag = false;
bool isSystemActive = false; // Chair won't move until clap is heard
unsigned long lastClapTime = 0;

// Motor Driver Pins
#define FL_IN1 26
#define FL_IN2 27
#define FR_IN3 14
#define FR_IN4 13
#define BL_IN1 25
#define BL_IN2 33
#define BR_IN3 16
#define BR_IN4 17

#define ENA_1 5
#define ENB_1 18
#define ENA_2 19
#define ENB_2 23

// Ultrasonic Sensor Pins
#define TRIG_FRONT 4
#define ECHO_FRONT 34

#define TRIG_LEFT  32
#define ECHO_LEFT  35

#define TRIG_RIGHT 15
#define ECHO_RIGHT 12

#define OBSTACLE_STOP_DIST 25.0 // 25cm Safety Limit

// MPU6050
#define MPU_ADDR 0x68
#define GYRO_SCALE 65.5

float gyroBiasZ = 0.0;
float rotationAngle = 0.0;
unsigned long lastMicros = 0;

bool rotationActive = false;
bool rotationClockwise = true;
float targetAngle = 0.0;
int maxSpeed = 120;
unsigned long startTime = 0;
unsigned long calculatedTimeout = 10000;

// Gyro Live Stall Tracking Variables
float lastCheckAngle = -999.0;
unsigned long lastAngleChangeTime = 0;

// State logic for safety obstacle check during Forward movement
bool isDrivingForward = false;
unsigned long lastUltrasonicCheck = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// Interrupt Service Routine (Runs instantly on Sound Pulse)
void IRAM_ATTR soundISR() {
  unsigned long currentTime = millis();
  // 500ms Debounce to prevent multiple false hits
  if (currentTime - lastClapTime > 500) {
    clapDetectedFlag = true;
    lastClapTime = currentTime;
  }
}

void setupSoundSensor() {
  pinMode(SOUND_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SOUND_SENSOR_PIN), soundISR, FALLING);
}

void processClapSignal() {
  if (clapDetectedFlag && !isSystemActive) {
    clapDetectedFlag = false;
    isSystemActive = true;

    Serial.println("\n==================================================");
    Serial.println(" [SUCCESS]: INTERRUPT CLAP DETECTED! ACTIVATING... ");
    Serial.println("==================================================");

    // Send MQTT command 3 times safely
    for (int i = 0; i < 3; i++) {
      client.publish(TOPIC_STATUS, "CLAP_DETECTED");
      delay(40);
    }
  }
}

// ============================================================
// MANUAL RESET SWITCH CHECK (GPIO 0)
// ============================================================
void checkResetButton() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    delay(50); // Debounce delay
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
      
      // Reset logic state
      isSystemActive = false;
      clapDetectedFlag = false;
      rotationActive = false;
      stopMotors();

      Serial.println("\n==================================================");
      Serial.println(" [MANUAL RESET]: GPIO 0 BUTTON PRESSED! READY AGAIN ");
      Serial.println("==================================================");

      client.publish(TOPIC_STATUS, "MANUAL_RESET");

      // Wait until button is released to prevent loop repeating
      while (digitalRead(RESET_BUTTON_PIN) == LOW) { delay(10); }
    }
  }
}

// ============================================================
// ULTRASONIC SENSOR READ FUNCTION
// ============================================================
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

// ============================================================
// MPU6050 FUNCTIONS
// ============================================================
void verifyAndWakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(MPU_ADDR, 1, true);
    if (Wire.available()) {
      byte pwr = Wire.read();
      if (pwr != 0x00) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
      }
    }
  }
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(MPU_ADDR, 2, true) < 2) return 0;
  return (Wire.read() << 8) | Wire.read();
}

void calibrateGyro() {
  Serial.println("\n--- Calibrating MPU6050 ---");
  long total = 0;
  for (int i = 0; i < 300; i++) {
    total += readGyroZRaw();
    delay(2);
  }
  gyroBiasZ = (total / 300.0) / GYRO_SCALE;
}

void updateAngle() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;

  if (dt <= 0) return;
  if (dt > 0.10) { lastMicros = now; return; }
  
  lastMicros = now;
  float gz = (readGyroZRaw() / GYRO_SCALE) - gyroBiasZ;
  rotationAngle += (gz * dt);
}

// ============================================================
// MOTOR FUNCTIONS
// ============================================================
void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  ledcWrite(ENA_1, speed); ledcWrite(ENB_1, speed);
  ledcWrite(ENA_2, speed); ledcWrite(ENB_2, speed);
}

void stopMotors() {
  isDrivingForward = false;
  setMotorSpeed(255);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN3, HIGH); digitalWrite(FR_IN4, HIGH);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN3, HIGH); digitalWrite(BR_IN4, HIGH);
  delay(60);
  setMotorSpeed(0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW); digitalWrite(FR_IN4, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN3, LOW); digitalWrite(BR_IN4, LOW);
}

void driveForward(int speed) {
  rotationActive = false; // Disable MPU Rotation loop
  
  // 3 Sensors Safety Check before starting move
  float f = getDistance(TRIG_FRONT, ECHO_FRONT);
  delayMicroseconds(500);
  float l = getDistance(TRIG_LEFT, ECHO_LEFT);
  delayMicroseconds(500);
  float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (f < OBSTACLE_STOP_DIST || l < OBSTACLE_STOP_DIST || r < OBSTACLE_STOP_DIST) {
    stopMotors();
    Serial.printf("[SAFETY BLOCK]: Obstacle detected (F:%.1f, L:%.1f, R:%.1f cm)\n", f, l, r);
    client.publish(TOPIC_STATUS, "OBSTACLE");
    return;
  }

  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(FR_IN3, HIGH); digitalWrite(FR_IN4, LOW);
  digitalWrite(BR_IN3, HIGH); digitalWrite(BR_IN4, LOW);
  
  setMotorSpeed(speed);
  isDrivingForward = true;
  Serial.printf("DRIVING FORWARD at Speed: %d\n", speed);
}

void rotateCW_Pins() {
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(FR_IN3, LOW);  digitalWrite(FR_IN4, HIGH);
  digitalWrite(BR_IN3, LOW);  digitalWrite(BR_IN4, HIGH);
}

void rotateACW_Pins() {
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH);
  digitalWrite(BL_IN1, LOW);  digitalWrite(BL_IN2, HIGH);
  digitalWrite(FR_IN3, HIGH); digitalWrite(FR_IN4, LOW);
  digitalWrite(BR_IN3, HIGH); digitalWrite(BR_IN4, LOW);
}

void startRotation(bool clockwise, float degrees, int speed) {
  isDrivingForward = false;
  rotationActive = false;
  verifyAndWakeMPU();
  
  rotationClockwise = clockwise;
  targetAngle = fabs(degrees);
  maxSpeed = constrain(speed, 85, 180);
  calculatedTimeout = (unsigned long)((targetAngle / 10.0) * 1000.0) + 5000;
  
  rotationAngle = 0.0;
  lastCheckAngle = -999.0;
  lastAngleChangeTime = millis();
  lastMicros = micros();
  startTime = millis();

  if (rotationClockwise) rotateCW_Pins();
  else rotateACW_Pins();

  rotationActive = true;
  Serial.printf("\nSTART %s: Target = %.1f deg\n", clockwise ? "CW" : "ACW", targetAngle);
}

void processRotation() {
  if (!rotationActive) return;

  updateAngle();
  float current = fabs(rotationAngle);
  float remaining = targetAngle - current;

  // Track live angle movement
  if (fabs(current - lastCheckAngle) > 0.2) {
    lastCheckAngle = current;
    lastAngleChangeTime = millis();
  }

  // Live Gyro Stall Check: If angle freezes for 400ms mid-turn -> Emergency Safety Stop
  if (millis() - startTime > 500 && (millis() - lastAngleChangeTime > 400)) {
    stopMotors();
    rotationActive = false;
    Serial.println("SAFETY STOP: Gyro frozen mid-turn!");
    client.publish(TOPIC_STATUS, "SENSOR_STALL_ERROR");
    return;
  }

  if (millis() - startTime >= calculatedTimeout) {
    stopMotors();
    rotationActive = false;
    Serial.printf("TIMEOUT! Measured: %.2f deg\n", current);
    client.publish(TOPIC_STATUS, "TIMEOUT");
    return;
  }

  if (remaining <= 3.5) {
    stopMotors();
    rotationActive = false;
    Serial.printf("TARGET REACHED! Final Measured: %.2f deg\n", current);
    client.publish(TOPIC_STATUS, "DONE");
    return;
  }

  int dynamicPWM = constrain(remaining * 3.5, 90, maxSpeed);
  setMotorSpeed(dynamicPWM);
}

// Check front obstacle non-blockingly during forward drive
void monitorForwardObstacle() {
  if (!isDrivingForward) return;

  if (millis() - lastUltrasonicCheck >= 40) { // Check every 40ms
    lastUltrasonicCheck = millis();
    
    float f = getDistance(TRIG_FRONT, ECHO_FRONT);
    delayMicroseconds(500);
    float l = getDistance(TRIG_LEFT, ECHO_LEFT);
    delayMicroseconds(500);
    float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    if (f < OBSTACLE_STOP_DIST || l < OBSTACLE_STOP_DIST || r < OBSTACLE_STOP_DIST) {
      stopMotors();
      Serial.printf("[SAFETY STOP]: Obstacle at F:%.1f, L:%.1f, R:%.1f cm!\n", f, l, r);
      client.publish(TOPIC_STATUS, "OBSTACLE");
    }
  }
}

// ============================================================
// MQTT CALLBACK
// ============================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  
  Serial.print("[MQTT RECEIVED]: ");
  Serial.println(msg);

  // 1. FORWARD COMMAND
  if (msg.startsWith("FORWARD:")) {
    int colon = msg.indexOf(':');
    int speed = msg.substring(colon + 1).toInt();
    driveForward(speed);
  }
  // 2. ROTATE CLOCKWISE
  else if (msg.startsWith("ROTATE_CW:")) {
    int c1 = msg.indexOf(':');
    int c2 = msg.indexOf(':', c1 + 1);
    startRotation(true, msg.substring(c1 + 1, c2).toFloat(), msg.substring(c2 + 1).toInt());
  } 
  // 3. ROTATE ANTICLOCKWISE
  else if (msg.startsWith("ROTATE_ACW:")) {
    int c1 = msg.indexOf(':');
    int c2 = msg.indexOf(':', c1 + 1);
    startRotation(false, msg.substring(c1 + 1, c2).toFloat(), msg.substring(c2 + 1).toInt());
  } 
  // 4. STOP MOTORS
  else if (msg == "STOP") {
    rotationActive = false;
    stopMotors();
  }
  // 5. QUERY ALL ULTRASONIC SENSOR DISTANCES
  else if (msg == "GET_SENSORS") {
    float f = getDistance(TRIG_FRONT, ECHO_FRONT);
    delay(20);
    float l = getDistance(TRIG_LEFT, ECHO_LEFT);
    delay(20);
    float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    char buf[64];
    snprintf(buf, sizeof(buf), "FRONT:%.1f,LEFT:%.1f,RIGHT:%.1f", f, l, r);
    client.publish(TOPIC_STATUS, buf);
    Serial.println(buf);
  }
  // 6. SOFTWARE RESET COMMAND FROM PYTHON
  else if (msg == "RESET_CLAP") {
    isSystemActive = false;
    clapDetectedFlag = false;
    stopMotors();
    Serial.println("\n[MQTT RESET]: Chair reset and ready for next clap!");
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Reset Button Setup on GPIO 0
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  setupSoundSensor();

  // Motor Pins Setup
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT); pinMode(FR_IN4, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT);
  pinMode(BR_IN3, OUTPUT); pinMode(BR_IN4, OUTPUT);

  ledcAttach(ENA_1, 1000, 8); ledcAttach(ENB_1, 1000, 8);
  ledcAttach(ENA_2, 1000, 8); ledcAttach(ENB_2, 1000, 8);
  stopMotors();

  // Ultrasonic Pins Setup
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_LEFT,  LOW);
  digitalWrite(TRIG_RIGHT, LOW);

  // Wire & MPU Setup
  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  Wire.setClock(100000);

  verifyAndWakeMPU();
  calibrateGyro();

  // WiFi Setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    if (client.connect("ESP32Chair")) client.subscribe(TOPIC_COMMANDS);
  }
  client.loop();
  
  processClapSignal();       // Sound Sensor Check
  checkResetButton();        // Push Button Check on GPIO 0
  processRotation();        // Motor angle rotation with live stall safety
  monitorForwardObstacle(); // Non-blocking forward obstacle check
}

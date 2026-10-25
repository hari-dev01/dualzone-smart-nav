/*
  DUAL-ZONE INTEGRATED LOGIC DEMO — Wokwi single-ESP32 project
  ----------------------------------------------------------------
  This combines test inputs from three separate wearable models in ONE
  simulator board. It exercises the shared decision logic, NOT the actual
  cross-board ESP-NOW radio, real ToF optics, or camera/AI perception.

  Simulated inputs:
    Pot 1 (GPIO34): cane ToF distance, 5..80 cm
    Pot 2 (GPIO35): simulated cane pitch, -60..+60 degrees
    Pot 3 (GPIO32): camera-derived object range, 0.5..8 m
    Pot 4 (GPIO33): lateral offset from walking line, -2.5..+2.5 m
    Pot 5 (GPIO36): relative closing speed, -1..+3 m/s
    Pot 6 (GPIO39): perception confidence, 0..100%
    3 x HC-SR04: left/front/right belt zones
    MPU6050: I2C motion telemetry only; pitch in this demo is controlled by
             Pot 2 so the ground-hazard behaviour can be tested interactively.

  LEDs:
    Green = no high-priority hazard
    Yellow = simulated raised/uneven surface
    Red = possible drop-off OR predicted path conflict (highest priority)
    Left / Front / Right = belt-zone haptic stand-ins
    Blue = low perception confidence (unknown, not clear)

  Physical hardware reminder: HC-SR04 ECHO is 5 V; divide/level-shift to 3.3 V.
  Do not connect real vibration motors directly to ESP32 GPIO pins.
*/
#include <Wire.h>
#include <math.h>

// Cane simulation inputs
constexpr int PIN_CANE_RANGE = 34;
constexpr int PIN_CANE_PITCH = 35;

// Perception-unit simulated outputs from a camera tracker
constexpr int PIN_OBJ_RANGE = 32;
constexpr int PIN_OBJ_LATERAL = 33;
constexpr int PIN_CLOSING_SPEED = 36;
constexpr int PIN_CONFIDENCE = 39;

// Ultrasonic zone pins: trigger / echo
struct Zone {
  const char *name;
  int trig;
  int echo;
  int led;
  bool side;
  float previousCm = -1.0f;
  unsigned long previousMs = 0;
  unsigned long lastBlinkMs = 0;
  bool ledState = false;
};
Zone zones[3] = {
  {"left",  5, 18, 13, true},
  {"front", 19, 23, 14, false},
  {"right", 16, 17, 4, true}
};

// Status LEDs
constexpr int LED_SAFE = 25;
constexpr int LED_RAISED = 26;
constexpr int LED_PRIORITY = 27;
constexpr int LED_UNCERTAIN = 15;

// MPU6050 I2C telemetry
constexpr uint8_t MPU_ADDR = 0x68;
bool mpuReady = false;

constexpr float DROP_DELTA_CM = 15.0f;
constexpr float RAISED_DELTA_CM = 10.0f;
constexpr int CONFIRM_READS = 3;
constexpr float FAR_CM = 183.0f;
constexpr float MEDIUM_CM = 100.0f;
constexpr float NEAR_CM = 50.0f;
constexpr float SIDE_CLOSE_SPEED_CM_S = 5.0f;
constexpr float PATH_HALF_WIDTH_M = 0.65f;
constexpr float TTC_WARNING_S = 3.0f;
constexpr float MIN_CONFIDENCE = 0.60f;

float caneBaselineCm = -1.0f;
int dropCount = 0;
int raisedCount = 0;
bool dropAlert = false;
unsigned long lastReportMs = 0;
bool priorityLedState = false;
unsigned long lastPriorityBlinkMs = 0;

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float readCaneRangeCm() {
  return 5.0f + (analogRead(PIN_CANE_RANGE) / 4095.0f) * 75.0f;
}

float readSimPitchDeg() {
  return -60.0f + (analogRead(PIN_CANE_PITCH) / 4095.0f) * 120.0f;
}

float readDistanceOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return 450.0f; // no echo treated as out of range
  float cm = duration * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 450.0f) return 450.0f;
  return cm;
}

float readMedianDistance(Zone &z) {
  float a = readDistanceOnce(z.trig, z.echo);
  delay(4);
  float b = readDistanceOnce(z.trig, z.echo);
  delay(4);
  float c = readDistanceOnce(z.trig, z.echo);
  return median3(a, b, c);
}

float updateClosingSpeed(Zone &z, float cm, unsigned long nowMs) {
  if (z.previousCm < 0.0f || nowMs <= z.previousMs) {
    z.previousCm = cm;
    z.previousMs = nowMs;
    return 0.0f;
  }
  float dt = (nowMs - z.previousMs) / 1000.0f;
  float closing = (z.previousCm - cm) / dt; // positive = getting closer
  z.previousCm = cm;
  z.previousMs = nowMs;
  return closing;
}

unsigned long pulseInterval(float cm) {
  if (cm <= NEAR_CM) return 120;
  if (cm <= MEDIUM_CM) return 300;
  if (cm <= FAR_CM) return 700;
  return 0;
}

void updateZoneLed(Zone &z, float cm, float closingCmS, unsigned long nowMs) {
  bool inRange = cm <= FAR_CM;
  bool stableSide = z.side && cm > NEAR_CM && closingCmS < SIDE_CLOSE_SPEED_CM_S;
  unsigned long interval = (inRange && !stableSide) ? pulseInterval(cm) : 0;
  if (interval == 0) {
    z.ledState = false;
    digitalWrite(z.led, LOW);
    return;
  }
  if (nowMs - z.lastBlinkMs >= interval) {
    z.lastBlinkMs = nowMs;
    z.ledState = !z.ledState;
    digitalWrite(z.led, z.ledState ? HIGH : LOW);
  }
}

bool checkMpu() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

bool readMpuAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6, true) != 6) return false;
  int16_t rawAx = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t rawAy = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t rawAz = (int16_t)((Wire.read() << 8) | Wire.read());
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  return true;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PIN_CANE_RANGE, INPUT);
  pinMode(PIN_CANE_PITCH, INPUT);
  pinMode(PIN_OBJ_RANGE, INPUT);
  pinMode(PIN_OBJ_LATERAL, INPUT);
  pinMode(PIN_CLOSING_SPEED, INPUT);
  pinMode(PIN_CONFIDENCE, INPUT);

  for (Zone &z : zones) {
    pinMode(z.trig, OUTPUT);
    pinMode(z.echo, INPUT);
    pinMode(z.led, OUTPUT);
    digitalWrite(z.trig, LOW);
    digitalWrite(z.led, LOW);
  }
  pinMode(LED_SAFE, OUTPUT);
  pinMode(LED_RAISED, OUTPUT);
  pinMode(LED_PRIORITY, OUTPUT);
  pinMode(LED_UNCERTAIN, OUTPUT);
  digitalWrite(LED_SAFE, LOW);
  digitalWrite(LED_RAISED, LOW);
  digitalWrite(LED_PRIORITY, LOW);
  digitalWrite(LED_UNCERTAIN, LOW);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // wake MPU6050
  mpuReady = (Wire.endTransmission() == 0);

  Serial.println();
  Serial.println("DUAL-ZONE INTEGRATED WOKWI DEMO");
  Serial.println("One ESP32 combines A/B/C LOGIC ONLY; no actual wireless link or camera.");
  Serial.println("Set cane range + pitch knobs to a normal floor state BEFORE calibration.");
  Serial.println("Then change one input at a time and watch LEDs + JSON Serial output.");
  Serial.print("MPU6050 I2C: ");
  Serial.println(mpuReady ? "OK" : "NOT FOUND");
  delay(250);

  const int N = 25;
  float sum = 0.0f;
  for (int i = 0; i < N; ++i) {
    float d = readCaneRangeCm();
    float theta = readSimPitchDeg();
    sum += d * cosf(theta * PI / 180.0f);
    delay(30);
  }
  caneBaselineCm = sum / N;
  digitalWrite(LED_SAFE, HIGH);
  Serial.print("Cane baseline h0_cm=");
  Serial.println(caneBaselineCm, 2);
}

void loop() {
  unsigned long nowMs;
  float zoneCm[3];
  float zoneClosing[3];

  // Read one ultrasonic at a time to reduce acoustic cross-talk.
  for (int i = 0; i < 3; ++i) {
    zoneCm[i] = readMedianDistance(zones[i]);
    nowMs = millis();
    zoneClosing[i] = updateClosingSpeed(zones[i], zoneCm[i], nowMs);
  }
  nowMs = millis();
  for (int i = 0; i < 3; ++i) {
    updateZoneLed(zones[i], zoneCm[i], zoneClosing[i], nowMs);
  }

  // Model A: simulated ToF range + angle; baseline set on boot.
  float caneD = readCaneRangeCm();
  float pitchDeg = readSimPitchDeg();
  float projectedHeight = caneD * cosf(pitchDeg * PI / 180.0f);
  float deltaHeight = projectedHeight - caneBaselineCm;
  if (deltaHeight > DROP_DELTA_CM) ++dropCount; else dropCount = 0;
  if (deltaHeight < -RAISED_DELTA_CM) ++raisedCount; else raisedCount = 0;
  if (dropCount >= CONFIRM_READS) dropAlert = true;
  if (dropAlert && deltaHeight < 8.0f) dropAlert = false;
  bool raisedAlert = raisedCount >= CONFIRM_READS;

  // Model C: synthetic camera tracker outputs supplied by potentiometers.
  float objectRangeM = 0.5f + analogRead(PIN_OBJ_RANGE) / 4095.0f * 7.5f;
  float lateralM = -2.5f + analogRead(PIN_OBJ_LATERAL) / 4095.0f * 5.0f;
  float closingMs = -1.0f + analogRead(PIN_CLOSING_SPEED) / 4095.0f * 4.0f;
  float confidence = analogRead(PIN_CONFIDENCE) / 4095.0f;
  float ttcS = (closingMs > 0.05f) ? objectRangeM / closingMs : 999.0f;
  bool unknown = confidence < MIN_CONFIDENCE;
  bool pathConflict = !unknown && fabsf(lateralM) <= PATH_HALF_WIDTH_M &&
                      closingMs > 0.05f && ttcS <= TTC_WARNING_S;

  // Priority policy: drop-off or predicted path conflict is highest priority.
  bool priority = dropAlert || pathConflict;
  digitalWrite(LED_RAISED, (!priority && raisedAlert) ? HIGH : LOW);
  if (priority) {
    digitalWrite(LED_SAFE, LOW);
    if (nowMs - lastPriorityBlinkMs >= 100) {
      lastPriorityBlinkMs = nowMs;
      priorityLedState = !priorityLedState;
      digitalWrite(LED_PRIORITY, priorityLedState ? HIGH : LOW);
    }
  } else {
    priorityLedState = false;
    digitalWrite(LED_PRIORITY, LOW);
    digitalWrite(LED_SAFE, raisedAlert ? LOW : HIGH);
  }
  digitalWrite(LED_UNCERTAIN, unknown ? HIGH : LOW);

  if (nowMs - lastReportMs >= 500) {
    lastReportMs = nowMs;
    float ax = 0, ay = 0, az = 0;
    bool imuOk = mpuReady && readMpuAccel(ax, ay, az);

    // One JSON object per line gives the team a common integration/debug format.
    Serial.print("{\"cane_cm\":"); Serial.print(caneD, 1);
    Serial.print(",\"pitch_sim_deg\":"); Serial.print(pitchDeg, 1);
    Serial.print(",\"projected_h_cm\":"); Serial.print(projectedHeight, 1);
    Serial.print(",\"delta_h_cm\":"); Serial.print(deltaHeight, 1);
    Serial.print(",\"dropoff\":"); Serial.print(dropAlert ? "true" : "false");
    Serial.print(",\"raised\":"); Serial.print(raisedAlert ? "true" : "false");
    Serial.print(",\"belt_left_cm\":"); Serial.print(zoneCm[0], 0);
    Serial.print(",\"belt_front_cm\":"); Serial.print(zoneCm[1], 0);
    Serial.print(",\"belt_right_cm\":"); Serial.print(zoneCm[2], 0);
    Serial.print(",\"object_range_m\":"); Serial.print(objectRangeM, 2);
    Serial.print(",\"lateral_m\":"); Serial.print(lateralM, 2);
    Serial.print(",\"closing_mps\":"); Serial.print(closingMs, 2);
    Serial.print(",\"ttc_s\":");
    if (ttcS >= 999.0f) Serial.print("null"); else Serial.print(ttcS, 2);
    Serial.print(",\"confidence\":"); Serial.print(confidence, 2);
    Serial.print(",\"path_conflict\":"); Serial.print(pathConflict ? "true" : "false");
    Serial.print(",\"unknown\":"); Serial.print(unknown ? "true" : "false");
    Serial.print(",\"priority\":"); Serial.print(priority ? "true" : "false");
    Serial.print(",\"mpu_ok\":"); Serial.print(imuOk ? "true" : "false");
    Serial.println("}");
  }
  delay(10);
}

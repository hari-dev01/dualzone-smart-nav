/*
  DualZone Smart Assistive Navigation - ONE-ESP32 TEST BENCH
  Educational simulation only. NOT a safety device.

  01 CANE : POT1 range, POT2 pitch, POT3 lateral offset, MPU6050
  02 ESP32: filtering, confirmation, fusion, arbitration
  03 BELT : 3x HC-SR04, 4 LED+buzzer PAIRS (1 GPIO each), 1 CLEAR LED

  Serial Monitor: send 'c' to calibrate the cane baseline (range + pitch).
*/
#include <Arduino.h>
#include <Wire.h>

// ---------------- Pins ----------------
const uint8_t PIN_POT_RANGE = 15, PIN_POT_PITCH = 2, PIN_POT_LAT = 4;
const uint8_t PIN_SDA = 21, PIN_SCL = 22, MPU_ADDR = 0x68;

const uint8_t PIN_STATUS = 13;                       // SYSTEM CLEAR LED
const uint8_t PAIR_PIN[4] = {12, 14, 27, 26};        // 0 LEFT, 1 FRONT, 2 RIGHT, 3 GROUND
const uint8_t TRIG_PIN[3] = {25, 33, 32};            // W, N, E
const uint8_t ECHO_PIN[3] = {35, 34, 39};            // W, N, E (39 = VN)

// ---------------- Tunables (heuristic) ----------------
const float    MAX_CM = 400, CAUTION_CM = 150, DANGER_CM = 70, NARROW_CM = 100;
const float    NOISE_CM = 40;            // reading spread above this = low confidence
const uint8_t  CONFIRM_HITS = 3;         // consecutive danger readings to confirm
const int      LAT_DEADBAND = 25;        // +-% of path centre
const float    RANGE_MAX_CM = 120;       // POT1 maps to 0..120 cm ground range
const float    DROP_CM = 25, STEP_CM = 20, UNEVEN_CM = 12, UNEVEN_PITCH = 15;
const float    PITCH_MIN = 15, PITCH_MAX = 75;   // outside = cane held implausibly
const uint8_t  GROUND_HITS = 2;
const float    JERK_G = 1.5f, TILT_FALL_DEG = 80;
const uint32_t SENSE_MS = 100, PRINT_MS = 500;
const uint16_t PAIR_TONE_HZ[4] = {1800, 2400, 3000, 2000};

// Priority order: higher number wins when several rules ask for the same pair
enum PState : uint8_t { S_OFF, S_CAUTION, S_PENDING, S_STEP, S_DANGER, S_DROPOFF, S_BLOCKED, S_ALARM };
enum OutCmd : uint8_t { C_OFF, C_HIGH, C_TONE };
const char* SN[8] = {"OFF", "CAUTION", "PENDING", "STEP", "DANGER", "DROP-OFF", "BLOCKED", "ALARM"};
const char* PN[4] = {"L", "F", "R", "G"};

// ---------------- LEDC (arduino-esp32 v2 and v3) ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define PAIR_ATTACH(i)  ledcAttach(PAIR_PIN[i], 2000, 10)
  #define PAIR_TONE(i, f) ledcWriteTone(PAIR_PIN[i], f)
  #define PAIR_DUTY(i, d) ledcWrite(PAIR_PIN[i], d)
#else
  #define PAIR_ATTACH(i)  do { ledcSetup(i, 2000, 10); ledcAttachPin(PAIR_PIN[i], i); } while (0)
  #define PAIR_TONE(i, f) ledcWriteTone(i, f)
  #define PAIR_DUTY(i, d) ledcWrite(i, d)
#endif

// ---------------- State ----------------
float    hist[3][3], dist[3] = {MAX_CM, MAX_CM, MAX_CM}, spread[3] = {0, 0, 0};
uint8_t  hIdx = 0, hits[3] = {0, 0, 0};
float    rangeCm = 0, pitchDeg = 0, baseRange = 60, basePitch = 45;
int      latPct = 0;
float    tiltDeg = 0, jerkG = 0;
bool     mpuOk = false;
uint8_t  dropHits = 0, stepHits = 0;
uint32_t dropUntil = 0, alarmUntil = 0, lastSense = 0, lastPrint = 0;
PState   req[4] = {S_OFF, S_OFF, S_OFF, S_OFF};
OutCmd   lastCmd[4] = {C_OFF, C_OFF, C_OFF, C_OFF};
uint16_t lastFreq[4] = {0, 0, 0, 0};
bool     clearLed = true;

// ---------------- Helpers ----------------
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

void raise(uint8_t pair, PState s) { if (s > req[pair]) req[pair] = s; }

void applyOut(uint8_t i, OutCmd cmd, uint16_t freq) {
  if (cmd == lastCmd[i] && (cmd != C_TONE || freq == lastFreq[i])) return;
  lastCmd[i] = cmd; lastFreq[i] = freq;
  if (cmd == C_OFF)       PAIR_DUTY(i, 0);
  else if (cmd == C_HIGH) PAIR_DUTY(i, 1023);   // steady HIGH: LED on, buzzer silent
  else                    PAIR_TONE(i, freq);   // square wave: LED blinks + buzzer sounds
}

float readCm(uint8_t z) {
  digitalWrite(TRIG_PIN[z], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN[z], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN[z], LOW);
  unsigned long us = pulseIn(ECHO_PIN[z], HIGH, 25000UL);
  if (us == 0) return MAX_CM;
  float cm = us * 0.01715f;
  return cm > MAX_CM ? MAX_CM : cm;
}

bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

bool mpuRead(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;
  int16_t r[3];
  for (int i = 0; i < 3; i++) { uint8_t hi = Wire.read(); uint8_t lo = Wire.read(); r[i] = (int16_t)((hi << 8) | lo); }
  ax = r[0] / 16384.0f; ay = r[1] / 16384.0f; az = r[2] / 16384.0f;
  return true;
}

void calibrate() {
  baseRange = rangeCm; basePitch = pitchDeg; dropHits = stepHits = 0;
  Serial.printf(">> Calibrated: baseline range=%.0f cm, pitch=%.0f deg\n", baseRange, basePitch);
}

// ---------------- 01 CANE ----------------
void readCane(uint32_t now) {
  rangeCm  = analogRead(PIN_POT_RANGE) * RANGE_MAX_CM / 4095.0f;
  pitchDeg = analogRead(PIN_POT_PITCH) * 90.0f / 4095.0f;
  latPct   = map(analogRead(PIN_POT_LAT), 0, 4095, -100, 100);

  float ax, ay, az;
  if (mpuOk && mpuRead(ax, ay, az)) {
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    jerkG   = fabsf(mag - 1.0f);
    tiltDeg = atan2f(sqrtf(ax * ax + ay * ay), az) * 57.2958f;
    if (jerkG > JERK_G || tiltDeg > TILT_FALL_DEG) alarmUntil = now + 1000;
  }
}

// ---------------- 03 BELT sensing ----------------
void readBelt() {
  for (uint8_t z = 0; z < 3; z++) {
    hist[z][hIdx] = readCm(z);
    dist[z] = median3(hist[z][0], hist[z][1], hist[z][2]);
    float mn = min(hist[z][0], min(hist[z][1], hist[z][2]));
    float mx = max(hist[z][0], max(hist[z][1], hist[z][2]));
    spread[z] = mx - mn;
    delay(5);
  }
  hIdx = (hIdx + 1) % 3;
}

// ---------------- 02 ESP32 decision engine ----------------
void evaluate(uint32_t now) {
  for (uint8_t i = 0; i < 4; i++) req[i] = S_OFF;
  bool conf[3];

  // --- Rule group A: each ultrasonic -> its own zone pair ---
  for (uint8_t z = 0; z < 3; z++) {
    bool inDanger = dist[z] < DANGER_CM;
    if (inDanger) { if (hits[z] < 10) hits[z]++; } else hits[z] = 0;
    conf[z] = hits[z] >= CONFIRM_HITS;

    if (conf[z])                         raise(z, S_DANGER);    // confirmed: beep
    else if (inDanger)                   raise(z, S_PENDING);   // waiting for confirmation
    else if (dist[z] < CAUTION_CM)       raise(z, spread[z] > NOISE_CM ? S_PENDING : S_CAUTION);
  }

  // --- Rule group B: POT3 lateral offset steers the FRONT hazard to a side ---
  if (latPct < -LAT_DEADBAND || latPct > LAT_DEADBAND) {
    uint8_t side = (latPct < 0) ? 0 : 2;
    if (conf[1])                          raise(side, S_DANGER);   // FRONT + side
    else if (dist[1] < CAUTION_CM)        raise(side, S_CAUTION);
  }

  // --- Rule group C: combinations of ultrasonics ---
  if (dist[0] < NARROW_CM && dist[2] < NARROW_CM && dist[1] >= CAUTION_CM) raise(1, S_CAUTION);  // narrow passage
  if (conf[0] && conf[1] && conf[2]) for (uint8_t z = 0; z < 3; z++) raise(z, S_BLOCKED);        // corridor blocked

  // --- Rule group D: cane range (POT1) + pitch (POT2) -> GROUND pair ---
  float dev  = rangeCm - baseRange;
  float pdev = pitchDeg - basePitch;
  bool plausible = (pitchDeg >= PITCH_MIN && pitchDeg <= PITCH_MAX);
  if (!plausible) {
    dropHits = stepHits = 0;
    raise(3, S_PENDING);                                   // low confidence ground reading
  } else {
    dropHits = (dev >= DROP_CM)  ? min<uint8_t>(dropHits + 1, 10) : 0;
    stepHits = (dev <= -STEP_CM) ? min<uint8_t>(stepHits + 1, 10) : 0;
    if (dropHits >= GROUND_HITS) dropUntil = now + 1500;   // latch so it is not missed
    if (now < dropUntil)                    { raise(3, S_DROPOFF); raise(1, S_CAUTION); }  // FRONT LED joins
    else if (stepHits >= GROUND_HITS)       raise(3, S_STEP);
    else if (dev >= DROP_CM || dev <= -STEP_CM) raise(3, S_PENDING);
    else if (fabsf(dev) >= UNEVEN_CM || fabsf(pdev) >= UNEVEN_PITCH) raise(3, S_CAUTION);   // uneven
  }

  // --- Rule group E: IMU fall / jerk overrides everything ---
  if (now < alarmUntil) for (uint8_t i = 0; i < 4; i++) req[i] = S_ALARM;

  clearLed = true;
  for (uint8_t i = 0; i < 4; i++) if (req[i] != S_OFF) clearLed = false;
}

void drivePairs(uint32_t now) {
  for (uint8_t i = 0; i < 4; i++) {
    switch (req[i]) {
      case S_OFF:     applyOut(i, C_OFF, 0); break;
      case S_CAUTION: applyOut(i, C_HIGH, 0); break;
      case S_PENDING: applyOut(i, ((now % 1000) < 500) ? C_HIGH : C_OFF, 0); break;
      case S_STEP:    if ((now % 800) < 120) applyOut(i, C_TONE, 1500); else applyOut(i, C_OFF, 0); break;
      case S_DANGER:  if ((now % 400) < 200) applyOut(i, C_TONE, PAIR_TONE_HZ[i]); else applyOut(i, C_OFF, 0); break;
      case S_DROPOFF: if ((now % 160) < 80)  applyOut(i, C_TONE, 3500); else applyOut(i, C_OFF, 0); break;
      case S_BLOCKED: if ((now % 240) < 120) applyOut(i, C_TONE, 2800); else applyOut(i, C_OFF, 0); break;
      case S_ALARM:   if ((now % 500) < 250) applyOut(i, C_TONE, 1200); else applyOut(i, C_OFF, 0); break;
    }
  }
  digitalWrite(PIN_STATUS, clearLed ? HIGH : LOW);
}

void bootTest() {
  Serial.println("Self-test: CLEAR LED, then pairs L F R G");
  digitalWrite(PIN_STATUS, HIGH); delay(300); digitalWrite(PIN_STATUS, LOW);
  for (uint8_t i = 0; i < 4; i++) { applyOut(i, C_TONE, PAIR_TONE_HZ[i]); delay(200); applyOut(i, C_OFF, 0); delay(100); }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nDualZone ONE-ESP32 test bench (simulation only, NOT a safety device)");
  pinMode(PIN_STATUS, OUTPUT);
  for (uint8_t z = 0; z < 3; z++) {
    pinMode(TRIG_PIN[z], OUTPUT); digitalWrite(TRIG_PIN[z], LOW);
    pinMode(ECHO_PIN[z], INPUT);
    for (uint8_t k = 0; k < 3; k++) hist[z][k] = MAX_CM;
  }
  for (uint8_t i = 0; i < 4; i++) { PAIR_ATTACH(i); PAIR_DUTY(i, 0); }
  analogReadResolution(12);
  mpuOk = mpuInit();
  Serial.println(mpuOk ? "MPU6050 OK" : "MPU6050 NOT FOUND (fall detection disabled)");
  bootTest();
  readCane(millis());
  calibrate();                       // start with the current pot positions as 'normal floor'
  Serial.println("Send 'c' to re-calibrate the cane baseline.");
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) { char c = Serial.read(); if (c == 'c' || c == 'C') calibrate(); }

  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readCane(now);
    readBelt();
    evaluate(now);
  }
  drivePairs(now);

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("US L/F/R=%3.0f/%3.0f/%3.0f cm | range=%3.0f(dev %+4.0f) pitch=%2.0f lat=%+4d%% tilt=%4.1f jerk=%.2fg | L:%s F:%s R:%s G:%s | CLEAR=%d\n",
                  dist[0], dist[1], dist[2], rangeCm, rangeCm - baseRange, pitchDeg, latPct, tiltDeg, jerkG,
                  SN[req[0]], SN[req[1]], SN[req[2]], SN[req[3]], clearLed);
  }
}

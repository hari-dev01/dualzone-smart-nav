/*
  DualZone Smart Assistive Navigation - ONE-ESP32 TEST BENCH (v3)
  Educational simulation only. NOT a safety device.

  01 CANE : POT1 handle grip, POT2 pitch (0-180 deg), POT3 lateral offset (0-50 cm), MPU6050
  02 ESP32: filtering, confirmation, arbitration
  03 BELT : 3x HC-SR04, 4 LED+buzzer PAIRS (1 GPIO each), 1 SYSTEM-CLEAR LED

  Cane rules
    POT1 > 50 %            -> handle held: every LED solid (hazards still pre-empt their own pair)
    h = offset * cos(pitch)
    h > 15 cm              -> GROUND pair blinks + buzzer, buzzer pitch rises with h
                              (15 cm = lowest tone, 50 cm = highest tone)
  Belt rules
    Each ultrasonic zone drives ONLY its own pair (W->LEFT, N->FRONT, E->RIGHT).
*/
#include <Arduino.h>
#include <Wire.h>

// ---------------- Pins ----------------
const uint8_t PIN_POT_GRIP = 15, PIN_POT_PITCH = 2, PIN_POT_OFFSET = 4;
const uint8_t PIN_SDA = 21, PIN_SCL = 22, MPU_ADDR = 0x68;

const uint8_t PIN_STATUS  = 13;                      // SYSTEM CLEAR LED
const uint8_t PAIR_PIN[4] = {12, 14, 27, 26};        // 0 LEFT, 1 FRONT, 2 RIGHT, 3 GROUND
const uint8_t PAIR_CH[4]  = {0, 2, 4, 6};            // even LEDC channels = one timer per pair (no shared tone)
const uint8_t TRIG_PIN[3] = {25, 33, 32};            // W, N, E
const uint8_t ECHO_PIN[3] = {35, 34, 39};            // W, N, E (39 = VN)

// ---------------- Tunables ----------------
// Belt (ultrasonic)
const float    MAX_CM = 400, CAUTION_CM = 150, DANGER_CM = 70, NARROW_CM = 100;
const float    NOISE_CM = 40;            // reading spread above this = low confidence
const uint8_t  CONFIRM_HITS = 3;         // consecutive danger readings to confirm

// Cane
const float    GRIP_ON_PCT = 50, GRIP_OFF_PCT = 45;      // handle detect (small hysteresis)
const float    PITCH_MAX_DEG = 180;                      // POT2 full scale
const float    OFFSET_MAX_CM = 50;                       // POT3 full scale
const float    H_LIMIT_CM = 15, H_FULL_CM = 50;          // alarm threshold / top of tone scale
const uint16_t GROUND_F_MIN = 600, GROUND_F_MAX = 3000;  // Hz at h = 15 cm / h >= 50 cm
const uint8_t  GROUND_HITS = 2;                          // consecutive samples above limit
const float    JERK_G = 1.5f, TILT_FALL_DEG = 80;        // IMU fall / jerk

const uint32_t SENSE_MS = 100, PRINT_MS = 500;
const uint16_t PAIR_TONE_HZ[4] = {1800, 2400, 3000, 2000};

// Priority order: higher number wins when several rules ask for the same pair
enum PState : uint8_t { S_OFF, S_CAUTION, S_PENDING, S_DANGER, S_GROUND, S_BLOCKED, S_ALARM };
enum OutCmd : uint8_t { C_OFF, C_HIGH, C_TONE };
const char* SN[7] = {"OFF", "CAUTION", "PENDING", "DANGER", "GROUND-H", "BLOCKED", "ALARM"};

// ---------------- LEDC (arduino-esp32 v2 and v3) ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define PAIR_ATTACH(i)  ledcAttachChannel(PAIR_PIN[i], 2000, 10, PAIR_CH[i])
  #define PAIR_TONE(i, f) ledcWriteTone(PAIR_PIN[i], f)
  #define PAIR_DUTY(i, d) ledcWrite(PAIR_PIN[i], d)
#else
  #define PAIR_ATTACH(i)  do { ledcSetup(PAIR_CH[i], 2000, 10); ledcAttachPin(PAIR_PIN[i], PAIR_CH[i]); } while (0)
  #define PAIR_TONE(i, f) ledcWriteTone(PAIR_CH[i], f)
  #define PAIR_DUTY(i, d) ledcWrite(PAIR_CH[i], d)
#endif

// ---------------- State ----------------
float    hist[3][3], dist[3] = {MAX_CM, MAX_CM, MAX_CM}, spread[3] = {0, 0, 0};
uint8_t  hIdx = 0, hits[3] = {0, 0, 0};
float    gripPct = 0, pitchDeg = 0, offsetCm = 0, hCm = 0;
bool     gripOn = false;
uint16_t groundFreq = GROUND_F_MIN;
uint8_t  groundHits = 0;
float    tiltDeg = 0, jerkG = 0;
bool     mpuOk = false;
uint32_t alarmUntil = 0, lastSense = 0, lastPrint = 0;
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
  else                    PAIR_TONE(i, freq);   // square wave: LED lit + buzzer sounds
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

// ---------------- 01 CANE ----------------
void readCane(uint32_t now) {
  gripPct  = analogRead(PIN_POT_GRIP)   * 100.0f         / 4095.0f;   // POT1: 0..100 %
  pitchDeg = analogRead(PIN_POT_PITCH)  * PITCH_MAX_DEG  / 4095.0f;   // POT2: 0..180 deg
  offsetCm = analogRead(PIN_POT_OFFSET) * OFFSET_MAX_CM  / 4095.0f;   // POT3: 0..50 cm

  if (!gripOn && gripPct > GRIP_ON_PCT)       gripOn = true;
  else if (gripOn && gripPct < GRIP_OFF_PCT)  gripOn = false;

  hCm = offsetCm * cosf(pitchDeg * DEG_TO_RAD);                       // h = offset * cos(pitch)

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

  // --- Rule group A: each ultrasonic drives ONLY its own zone pair ---
  for (uint8_t z = 0; z < 3; z++) {
    bool inDanger = dist[z] < DANGER_CM;
    if (inDanger) { if (hits[z] < 10) hits[z]++; } else hits[z] = 0;
    conf[z] = hits[z] >= CONFIRM_HITS;

    if (conf[z])                         raise(z, S_DANGER);    // confirmed: beep
    else if (inDanger)                   raise(z, S_PENDING);   // waiting for confirmation
    else if (dist[z] < CAUTION_CM)       raise(z, spread[z] > NOISE_CM ? S_PENDING : S_CAUTION);
  }

  // --- Rule group B: combinations of ultrasonics ---
  if (dist[0] < NARROW_CM && dist[2] < NARROW_CM && dist[1] >= CAUTION_CM) raise(1, S_CAUTION);  // narrow passage
  if (conf[0] && conf[1] && conf[2]) for (uint8_t z = 0; z < 3; z++) raise(z, S_BLOCKED);        // corridor blocked

  // --- Rule group C: cane geometry h = offset * cos(pitch) -> GROUND pair ---
  if (hCm > H_LIMIT_CM) { if (groundHits < 10) groundHits++; } else groundHits = 0;
  if (groundHits >= GROUND_HITS) {
    raise(3, S_GROUND);
    float t = (min(hCm, H_FULL_CM) - H_LIMIT_CM) / (H_FULL_CM - H_LIMIT_CM);     // 0 at 15 cm .. 1 at 50 cm
    float f = GROUND_F_MIN + t * (GROUND_F_MAX - GROUND_F_MIN);
    groundFreq = 25 * (uint16_t)(f / 25.0f + 0.5f);                                // 25 Hz steps
  }

  // --- Rule group D: IMU fall / jerk overrides everything ---
  if (now < alarmUntil) for (uint8_t i = 0; i < 4; i++) req[i] = S_ALARM;

  clearLed = true;
  for (uint8_t i = 0; i < 4; i++) if (req[i] != S_OFF) clearLed = false;
}

void drivePairs(uint32_t now) {
  for (uint8_t i = 0; i < 4; i++) {
    switch (req[i]) {
      case S_OFF:     applyOut(i, gripOn ? C_HIGH : C_OFF, 0); break;      // handle held -> LED solid
      case S_CAUTION: applyOut(i, C_HIGH, 0); break;
      case S_PENDING: applyOut(i, ((now % 1000) < 500) ? C_HIGH : C_OFF, 0); break;
      case S_DANGER:  if ((now % 400) < 200) applyOut(i, C_TONE, PAIR_TONE_HZ[i]); else applyOut(i, C_OFF, 0); break;
      case S_GROUND:  if ((now % 500) < 250) applyOut(i, C_TONE, groundFreq);      else applyOut(i, C_OFF, 0); break;
      case S_BLOCKED: if ((now % 240) < 120) applyOut(i, C_TONE, 2800); else applyOut(i, C_OFF, 0); break;
      case S_ALARM:   if ((now % 500) < 250) applyOut(i, C_TONE, 1200); else applyOut(i, C_OFF, 0); break;
    }
  }
  // SYSTEM CLEAR LED: on when no hazard, or solid with everything else while the handle is held
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
  Serial.println("\nDualZone ONE-ESP32 test bench v3 (simulation only, NOT a safety device)");
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
  Serial.println("POT1 grip >50% = all LEDs solid | h = POT3 * cos(POT2) | h>15 cm = GROUND alarm");
}

void loop() {
  uint32_t now = millis();

  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readCane(now);
    readBelt();
    evaluate(now);
  }
  drivePairs(now);

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("US L/F/R=%3.0f/%3.0f/%3.0f cm | grip=%3.0f%%%s pitch=%5.1f off=%4.1fcm h=%+5.1fcm f=%4uHz | tilt=%4.1f jerk=%.2fg | L:%s F:%s R:%s G:%s | CLEAR=%d\n",
                  dist[0], dist[1], dist[2], gripPct, gripOn ? "(ON)" : "    ", pitchDeg, offsetCm, hCm,
                  (req[3] == S_GROUND) ? groundFreq : 0, tiltDeg, jerkG,
                  SN[req[0]], SN[req[1]], SN[req[2]], SN[req[3]], clearLed);
  }
}

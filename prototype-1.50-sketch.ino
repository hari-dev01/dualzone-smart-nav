/*
  DualZone Smart Assistive Navigation - ESP32 Test Bench (Wokwi simulation harness)
  Sections: CANE (MPU6050 + 3 pots)  ->  ESP32 (fusion + zone logic)  ->  BELT (3x HC-SR04, 3 LED+buzzer pairs)

  Simulation harness only - NOT a real mobility/safety device. Thresholds are heuristic.
  No external libraries needed (MPU6050 is read over raw I2C).

  Pin map
    Cane : POT1 position D15 | POT2 angle D2 | POT3 grip D4 | MPU6050 SDA D21, SCL D22
    Belt : WEST  TRIG D12 / ECHO D14 -> LEFT  pair (LED+buzzer) D13
           NORTH TRIG D26 / ECHO D25 -> FRONT pair (LED+buzzer) D27
           EAST  TRIG D32 / ECHO D35 -> RIGHT pair (LED+buzzer) D33
  Each pair shares ONE GPIO (330R+LED in parallel with the buzzer), so they always work as a pair:
    CLEAR  -> pin LOW   (LED off, buzzer silent)
    WARN   -> pin HIGH  (LED steady, buzzer silent)
    DANGER -> 2 kHz beeps (LED blinks with the buzzer; closer = faster)
*/
#include <Arduino.h>
#include <Wire.h>

// ---------------- pins ----------------
const int PIN_POT_POS = 15, PIN_POT_ANG = 2, PIN_POT_GRIP = 4;
const int PIN_SDA = 21, PIN_SCL = 22;

struct Zone {
  const char* name;
  int trig, echo, pair, ch;        // ch = LEDC channel (core 2.x only)
  float dist;                      // cm, <0 = no echo
  uint8_t hits;                    // consecutive "danger" readings
  uint8_t level;                   // 0 CLEAR, 1 WARN, 2 DANGER
  uint8_t mode;                    // current pair output mode
  bool attached;
};
Zone Z[3] = {
  {"LEFT(W)",  12, 14, 13, 0, -1, 0, 0, 255, false},
  {"FRONT(N)", 26, 25, 27, 1, -1, 0, 0, 255, false},
  {"RIGHT(E)", 32, 35, 33, 2, -1, 0, 0, 255, false},
};
enum { CLEAR = 0, WARN = 1, DANGER = 2 };
enum { M_OFF = 0, M_STEADY = 1, M_TONE = 2 };

// ---------------- tuning ----------------
const float DANGER_CM = 60, WARN_CM = 120, SWEEP_BONUS_CM = 30;
const uint8_t CONFIRM_N = 3, CONFIRM_N_SWING = 2;
const float DROPOFF_DEG = 70, FALLEN_TILT_DEG = 70, SWING_DPS = 40;
const int GRIP_MIN_PCT = 10;
const uint16_t BEEP_HZ = 2000;

// ---------------- pair output (LED + buzzer on one GPIO) ----------------
void pairSet(Zone& z, uint8_t mode, uint32_t hz = BEEP_HZ) {
  if (mode == M_TONE) {
    if (!z.attached) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcAttach(z.pair, BEEP_HZ, 8);
#else
      ledcSetup(z.ch, BEEP_HZ, 8);
      ledcAttachPin(z.pair, z.ch);
#endif
      z.attached = true;
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteTone(z.pair, hz);
#else
    ledcWriteTone(z.ch, hz);
#endif
  } else {
    if (z.attached) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcDetach(z.pair);
#else
      ledcDetachPin(z.pair);
#endif
      z.attached = false;
    }
    pinMode(z.pair, OUTPUT);
    digitalWrite(z.pair, mode == M_STEADY ? HIGH : LOW);
  }
  z.mode = mode;
}

// ---------------- ultrasonic ----------------
float readCm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 25000UL);   // ~4 m timeout
  if (us == 0) return -1;
  return us * 0.0343f / 2.0f;
}

// ---------------- MPU6050 (raw I2C) ----------------
const uint8_t MPU_ADDR = 0x68;
bool imuOk = false;
float tiltDeg = 0, gyroDps = 0;

bool imuBegin() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);               // wake up
  return Wire.endTransmission() == 0;
}
void imuRead() {
  if (!imuOk) return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return;
  int16_t v[7];
  for (int i = 0; i < 7; i++) { int hi = Wire.read(); int lo = Wire.read(); v[i] = (int16_t)((hi << 8) | lo); }
  float ax = v[0] / 16384.0f, ay = v[1] / 16384.0f, az = v[2] / 16384.0f;
  float gx = v[4] / 131.0f,   gy = v[5] / 131.0f,   gz = v[6] / 131.0f;
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag > 0.05f) tiltDeg = acosf(constrain(fabsf(az) / mag, 0.0f, 1.0f)) * 57.2958f;   // angle from vertical
  gyroDps = sqrtf(gx * gx + gy * gy + gz * gz);
}

// ---------------- helpers ----------------
int potPct(int pin) { return map(analogRead(pin), 0, 4095, 0, 100); }
const char* lvlName(uint8_t l) { return l == DANGER ? "DANGER" : (l == WARN ? "WARN" : "clear"); }

void bootDemo() {                                   // short LEFT -> FRONT -> RIGHT beep so each pair can be checked
  for (int i = 0; i < 3; i++) {
    pairSet(Z[i], M_TONE); delay(160);
    pairSet(Z[i], M_OFF);  delay(60);
  }
}

// ---------------- setup / loop ----------------
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  for (auto& z : Z) { pinMode(z.trig, OUTPUT); pinMode(z.echo, INPUT); pairSet(z, M_OFF); }
  pinMode(PIN_POT_POS, INPUT); pinMode(PIN_POT_ANG, INPUT); pinMode(PIN_POT_GRIP, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL);
  imuOk = imuBegin();
  Serial.println(F("\n=== DualZone Test Bench (ESP32 simulation harness) ==="));
  Serial.println(imuOk ? F("MPU6050 OK") : F("MPU6050 not found - tilt/swing disabled"));
  Serial.println(F("Drag the HC-SR04 distance sliders and the pots to test. NOT a real safety device."));
  bootDemo();
}

void loop() {
  // ---- CANE inputs ----
  imuRead();
  int posPct = potPct(PIN_POT_POS);                 // cane sweep position 0..100 %
  int angDeg = map(analogRead(PIN_POT_ANG), 0, 4095, 0, 90);   // cane tip angle 0..90 deg
  int gripPct = potPct(PIN_POT_GRIP);               // grip pressure 0..100 %
  int sweepZone = posPct < 33 ? 0 : (posPct > 66 ? 2 : 1);     // which belt zone the cane is probing
  bool swinging = gyroDps > SWING_DPS;
  bool dropOff = angDeg >= DROPOFF_DEG;
  bool caneDown = imuOk && tiltDeg >= FALLEN_TILT_DEG;
  bool gripLost = gripPct < GRIP_MIN_PCT;
  uint8_t need = swinging ? CONFIRM_N_SWING : CONFIRM_N;

  // ---- BELT sensing + zone logic ----
  for (int i = 0; i < 3; i++) {
    Zone& z = Z[i];
    z.dist = readCm(z.trig, z.echo);
    delay(15);                                       // let echoes die out between sensors
    float bonus = (i == sweepZone) ? SWEEP_BONUS_CM : 0;
    bool near = z.dist > 0 && z.dist <= DANGER_CM + bonus;
    bool mid  = z.dist > 0 && z.dist <= WARN_CM + bonus;
    z.hits = near ? min<int>(z.hits + 1, 250) : 0;
    z.level = (near && z.hits >= need) ? DANGER : ((mid || near) ? WARN : CLEAR);   // sound only when confirmed
  }
  if (dropOff) Z[1].level = DANGER;                  // drop-off -> FRONT pair
  if (caneDown || gripLost)                          // cane not in use -> raise every zone to at least WARN
    for (auto& z : Z) if (z.level < WARN) z.level = WARN;

  // ---- drive the LED+buzzer pairs ----
  unsigned long now = millis();
  for (auto& z : Z) {
    if (z.level == DANGER) {
      float d = z.dist > 0 ? z.dist : 0;
      unsigned long period = (unsigned long)constrain(map((long)d, 0, (long)DANGER_CM, 150, 450), 150, 450);
      bool on = (now % period) < 90;
      pairSet(z, on ? M_TONE : M_OFF);
    } else {
      pairSet(z, z.level == WARN ? M_STEADY : M_OFF);
    }
  }

  // ---- telemetry ----
  if (now - lastPrint >= 250) {
    lastPrint = now;
    for (auto& z : Z) {
      Serial.print(z.name); Serial.print(' ');
      if (z.dist > 0) { Serial.print(z.dist, 0); Serial.print("cm "); } else Serial.print("--- ");
      Serial.print(lvlName(z.level)); Serial.print(" | ");
    }
    Serial.printf("sweep=%d%% angle=%ddeg grip=%d%% tilt=%.0f swing=%s%s%s%s\n",
                  posPct, angDeg, gripPct, tiltDeg, swinging ? "Y" : "n",
                  dropOff ? " DROP-OFF" : "", caneDown ? " CANE-DOWN" : "", gripLost ? " GRIP-LOST" : "");
  }
}

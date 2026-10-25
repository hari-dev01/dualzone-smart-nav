/*
  DualZone Smart Assistive Navigation — clean single-ESP32 Wokwi test bench
  ========================================================================

  VISUAL SECTIONS:
    01 CANE / TEST INPUTS (left): 3 potentiometers + cane MPU6050
    02 ESP32 DECISION ENGINE (center)
    03 BELT (right): 3 ultrasonic zones + 7 state LEDs + 3 directional hazard buzzers

  IMPORTANT:
    - This is a single-ESP32 simulation harness, not two BLE-connected nodes.
    - Potentiometers simulate cane range, cane pitch, and perceived lateral offset.
    - Object range and closing speed are derived from the FRONT HC-SR04 over time.
    - Confidence is derived from sensor validity, sample history, and measurement jitter.
    - All reported height, TTC, confidence, and path-conflict values are heuristic demo
      metrics. They are not validated ground-truth measurements or safety guarantees.

  INDICATORS (one-hot through 74HC595):
    Q0 SYSTEM CLEAR (green; intentionally silent)
    Q1 RAISED / UNEVEN (yellow; visual-only state)
    Q2 DROP-OFF / PATH CONFLICT (red; critical buzzer)
    Q3 LEFT ZONE (blue; visual proximity state; conflict elevates red LED)
    Q4 FRONT ZONE (blue; visual proximity state; conflict elevates red LED)
    Q5 RIGHT ZONE (blue; visual proximity state; conflict elevates red LED)
    Q6 LOW CONFIDENCE (white/blue; visual-only; no buzzer)

  THREE BUZZER CHANNELS are sufficient:
    - Separate LEFT, FRONT, and RIGHT sounders.
    - Sound is enabled only for confirmed high-confidence path conflict or drop-off proxy.
    - Ordinary proximity, raised/uneven, clear, and low-confidence states are visual-only.
    - Buzzer circuits are electrically independent of LED/resistor branches. A buzzer is
      NEVER wired in series with an LED or through a 74HC595 output.

  REAL-HARDWARE NOTES:
    - Wokwi's piezo buzzer may be driven from a GPIO for simulation. For a physical
      buzzer that draws more than a GPIO can safely supply, use a separate NPN/MOSFET
      low-side driver and base/gate resistor; do not drive high-current buzzers directly.
    - Each HC-SR04 ECHO goes through its own 1kΩ/2kΩ divider in this diagram (5V -> ~3.3V).
    - Cane range and pitch are potentiometer test proxies; not real ToF measurements.
    - Drop-off logic is a projected-height proxy and requires on-screen test/calibration.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace Pin {
  // Exactly three cane/perception test potentiometers (ESP32 ADC1 inputs).
  constexpr uint8_t POT_CANE_RANGE = 34;
  constexpr uint8_t POT_CANE_PITCH = 35;
  constexpr uint8_t POT_LATERAL_OFFSET = 39;

  // Cane-mounted IMU (diagnostic reading in this combined demo).
  constexpr uint8_t MPU_SDA = 21;
  constexpr uint8_t MPU_SCL = 22;

  // 74HC595 drives LEDs only (DS, SHCP, STCP). No buzzer current passes through it.
  constexpr uint8_t LED_DATA = 4;
  constexpr uint8_t LED_CLOCK = 13;
  constexpr uint8_t LED_LATCH = 14;

  // One ultrasonic ping at a time. Echo pins are GPIO inputs.
  constexpr uint8_t ZONE_LEFT = 0;
  constexpr uint8_t ZONE_FRONT = 1;
  constexpr uint8_t ZONE_RIGHT = 2;
  constexpr uint8_t ZONE_COUNT = 3;
  constexpr uint8_t TRIG[ZONE_COUNT] = {18, 19, 23};
  constexpr uint8_t ECHO[ZONE_COUNT] = {32, 33, 36};

  // Three independent zone sounders; all LEDs are driven separately.
  constexpr uint8_t BUZZER_LEFT = 16;
  constexpr uint8_t BUZZER_FRONT = 17;
  constexpr uint8_t BUZZER_RIGHT = 25;
}

namespace Cfg {
  constexpr uint8_t MPU_ADDR = 0x68;
  constexpr uint32_t ECHO_TIMEOUT_US = 12000UL;
  constexpr uint32_t ZONE_SCAN_INTERVAL_MS = 35UL;
  constexpr uint32_t POT_INTERVAL_MS = 50UL;
  constexpr uint32_t IMU_INTERVAL_MS = 100UL;
  constexpr uint32_t LOG_INTERVAL_MS = 500UL;
  constexpr uint32_t MAX_SENSOR_AGE_MS = 650UL;
  constexpr uint32_t CALIBRATION_SAMPLE_MS = 20UL;
  constexpr uint8_t CALIBRATION_SAMPLES = 20;

  constexpr float POT_EMA_ALPHA = 0.25f;
  constexpr float RANGE_EMA_ALPHA = 0.40f;
  constexpr float SPEED_EMA_ALPHA = 0.30f;
  constexpr float NOISE_EMA_ALPHA = 0.25f;

  constexpr float DROP_DELTA_CM = 15.0f;
  constexpr float DROP_CLEAR_DELTA_CM = 8.0f;
  constexpr float RAISED_DELTA_CM = 10.0f;
  constexpr float RAISED_CLEAR_DELTA_CM = -5.0f;
  constexpr uint8_t CONFIRM_SAMPLES = 3;

  constexpr float ZONE_NEAR_CM = 100.0f;
  constexpr float ZONE_RELEASE_CM = 115.0f;
  constexpr uint8_t ZONE_CONFIRM_SAMPLES = 3;

  constexpr float PATH_HALF_WIDTH_M = 0.65f;
  constexpr float TTC_WARNING_S = 3.0f;
  constexpr float MIN_CLOSING_SPEED_MPS = 0.08f;
  constexpr float MIN_CONFIDENCE = 0.60f;
  constexpr float MAX_ABS_CLOSING_MPS = 5.0f;
  constexpr float MAX_NOISE_CM = 8.0f;
}

// State values intentionally remain plain integers: this avoids Arduino .ino
// auto-prototype ordering problems with custom enum types.
constexpr uint8_t STATE_CLEAR = 0;
constexpr uint8_t STATE_RAISED = 1;
constexpr uint8_t STATE_CRITICAL = 2;
constexpr uint8_t STATE_LEFT = 3;
constexpr uint8_t STATE_FRONT = 4;
constexpr uint8_t STATE_RIGHT = 5;
constexpr uint8_t STATE_LOW_CONFIDENCE = 6;
constexpr uint8_t STATE_COUNT = 7;
constexpr uint8_t NO_PIN = 255;

const char *const STATE_NAMES[STATE_COUNT] = {
  "SYSTEM CLEAR", "RAISED / UNEVEN", "DROP-OFF / PATH CONFLICT",
  "LEFT ZONE", "FRONT ZONE", "RIGHT ZONE", "LOW CONFIDENCE"
};

const char *const ZONE_NAMES[Pin::ZONE_COUNT] = {"left", "front", "right"};

struct ZoneData {
  float filteredCm;
  float closingMps;
  float noiseEmaCm;
  float history[3];
  uint8_t historyCount;
  uint8_t historyNext;
  uint8_t validStreak;
  uint8_t nearStreak;
  uint8_t pathCount;
  uint8_t pathExitCount;
  bool pathCandidate;
  bool pathConflict;
  float ttcS;
  uint32_t previousSampleMs;
  uint32_t lastValidMs;
  bool latestSampleValid;
};

struct RuntimeData {
  float caneRangeCm;
  float pitchDeg;
  float lateralM;
  float projectedHeightCm;
  float caneStepEmaCm;
  float caneConfidence;
  float baselineHeightCm;
  float deltaHeightCm;
  float frontRangeM;
  float closingMps;
  float ttcS;
  float frontTtcS;
  float frontConfidence;
  float overallConfidence;
  float accelX;
  float accelY;
  float accelZ;
  uint16_t rawRange;
  uint16_t rawPitch;
  uint16_t rawLateral;
  uint8_t dropCount;
  uint8_t dropExitCount;
  uint8_t raisedCount;
  uint8_t raisedExitCount;
  uint8_t pathCount;
  uint8_t pathExitCount;
  bool baselineReady;
  bool calibrationActive;
  bool mpuReady;
  bool mpuReadingValid;
  bool dropAlert;
  bool raisedAlert;
  bool pathCandidate;
  bool pathConflict;
  bool hazardConfirmed;
  float hazardConfidence;
  int8_t hazardZone;
  uint8_t indicator;
};

struct AudioRuntime {
  int8_t activeState;
  int8_t soundingPin;
  uint8_t phase;
  uint32_t phaseStartedMs;
  bool toneOn;
};

ZoneData zone[Pin::ZONE_COUNT];
RuntimeData data;
AudioRuntime audio = {-1, -1, 0, 0, false};

uint32_t lastPotMs = 0;
uint32_t lastImuMs = 0;
uint32_t lastScanMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastCalibrationSampleMs = 0;
uint32_t lastDecisionObservationMs = 0;

uint32_t nextZoneToScan = 0;
uint8_t decisionCandidate = STATE_LOW_CONFIDENCE;
uint8_t decisionCandidateCount = 0;
uint8_t calibrationCount = 0;
float calibrationSum = 0.0f;

char serialLine[40];
uint8_t serialLineLength = 0;
uint8_t lastWrittenLedState = 255;

float clampFloat(float value, float low, float high) {
  if (!isfinite(value)) return low;
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float ema(float previous, float current, float alpha) {
  if (!isfinite(current)) return previous;
  if (!isfinite(previous)) return current;
  return alpha * current + (1.0f - alpha) * previous;
}

float rawToCaneRange(uint16_t raw) {
  return 5.0f + (static_cast<float>(raw) / 4095.0f) * 75.0f;
}

float rawToPitch(uint16_t raw) {
  return -60.0f + (static_cast<float>(raw) / 4095.0f) * 120.0f;
}

float rawToLateral(uint16_t raw) {
  return -2.5f + (static_cast<float>(raw) / 4095.0f) * 5.0f;
}

float projectedHeight(float rangeCm, float pitchDeg) {
  return rangeCm * cosf(pitchDeg * DEG_TO_RAD);
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(Cfg::MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool beginMpu6050() {
  if (!writeMpuRegister(0x6B, 0x00)) return false;
  delay(10);

  Wire.beginTransmission(Cfg::MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(
        static_cast<uint8_t>(Cfg::MPU_ADDR),
        static_cast<uint8_t>(1),
        static_cast<uint8_t>(true)) != 1) {
    return false;
  }

  return Wire.read() == 0x68;
}

bool readMpuAcceleration() {
  Wire.beginTransmission(Cfg::MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(
        static_cast<uint8_t>(Cfg::MPU_ADDR),
        static_cast<uint8_t>(6),
        static_cast<uint8_t>(true)) != 6) {
    return false;
  }

  const int16_t rawAx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  const int16_t rawAy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  const int16_t rawAz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  data.accelX = rawAx / 16384.0f;
  data.accelY = rawAy / 16384.0f;
  data.accelZ = rawAz / 16384.0f;

  const float norm2 =
      data.accelX * data.accelX +
      data.accelY * data.accelY +
      data.accelZ * data.accelZ;

  return isfinite(norm2) && norm2 >= 0.15f && norm2 <= 4.0f;
}

float readDistanceCm(uint8_t index) {
  const uint8_t trig = Pin::TRIG[index];
  const uint8_t echo = Pin::ECHO[index];

  digitalWrite(trig, LOW);
  delayMicroseconds(3);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  const uint32_t durationUs = pulseIn(echo, HIGH, Cfg::ECHO_TIMEOUT_US);

  if (durationUs == 0) return NAN;

  const float cm = static_cast<float>(durationUs) / 58.0f;

  if (!isfinite(cm) || cm < 2.0f || cm > 400.0f) return NAN;

  return cm;
}

float medianHistory(uint8_t index) {
  float values[3];
  const uint8_t count = zone[index].historyCount;

  for (uint8_t i = 0; i < count; ++i) {
    values[i] = zone[index].history[i];
  }

  for (uint8_t i = 1; i < count; ++i) {
    float current = values[i];
    int j = static_cast<int>(i) - 1;

    while (j >= 0 && values[j] > current) {
      values[j + 1] = values[j];
      --j;
    }

    values[j + 1] = current;
  }

  if (count == 0) return NAN;
  if (count == 2) return 0.5f * (values[0] + values[1]);

  return values[count / 2];
}

void updateZone(uint8_t index, float rawCm, uint32_t nowMs) {
  ZoneData &z = zone[index];

  z.latestSampleValid = isfinite(rawCm);

  if (!z.latestSampleValid) {
    z.validStreak = 0;
    z.closingMps = 0.0f;
    z.pathCandidate = false;
    z.pathConflict = false;
    z.pathCount = 0;
    z.pathExitCount = 0;
    z.ttcS = NAN;

    if (nowMs - z.lastValidMs > Cfg::MAX_SENSOR_AGE_MS) {
      z.nearStreak = 0;
    }

    return;
  }

  const float previousFiltered = z.filteredCm;

  z.history[z.historyNext] = rawCm;
  z.historyNext = static_cast<uint8_t>((z.historyNext + 1U) % 3U);

  if (z.historyCount < 3) ++z.historyCount;

  const float medianCm = medianHistory(index);

  if (isfinite(previousFiltered)) {
    const float residual = fabsf(rawCm - medianCm);
    z.noiseEmaCm = ema(z.noiseEmaCm, residual, Cfg::NOISE_EMA_ALPHA);
  } else {
    z.noiseEmaCm = 0.0f;
  }

  z.filteredCm = ema(z.filteredCm, medianCm, Cfg::RANGE_EMA_ALPHA);

  if (isfinite(previousFiltered) &&
      z.previousSampleMs > 0 &&
      nowMs > z.previousSampleMs) {

    const float dtS = (nowMs - z.previousSampleMs) / 1000.0f;

    if (dtS >= 0.02f && dtS <= 1.0f) {
      const float rawClosing =
          (previousFiltered - z.filteredCm) / 100.0f / dtS;

      z.closingMps = ema(
          z.closingMps,
          clampFloat(rawClosing,
                     -Cfg::MAX_ABS_CLOSING_MPS,
                     Cfg::MAX_ABS_CLOSING_MPS),
          Cfg::SPEED_EMA_ALPHA);
    } else {
      z.closingMps = 0.0f;
    }
  } else {
    z.closingMps = 0.0f;
  }

  z.previousSampleMs = nowMs;
  z.lastValidMs = nowMs;

  if (z.validStreak < 250) ++z.validStreak;

  if (z.filteredCm <= Cfg::ZONE_NEAR_CM) {
    if (z.nearStreak < 250) ++z.nearStreak;
  } else if (z.filteredCm > Cfg::ZONE_RELEASE_CM) {
    z.nearStreak = 0;
  }
}

float zoneConfidence(uint8_t index, uint32_t nowMs) {
  const ZoneData &z = zone[index];

  if (z.lastValidMs == 0 ||
      nowMs - z.lastValidMs > Cfg::MAX_SENSOR_AGE_MS ||
      z.validStreak < 3) {
    return 0.35f;
  }

  const float noisePenalty = fminf(0.40f, z.noiseEmaCm * 0.025f);

  return clampFloat(0.92f - noisePenalty, 0.35f, 0.95f);
}

void updateOverallConfidence(uint32_t nowMs) {
  float minimumConfidence = 1.0f;
  bool allFresh = true;

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    if (zone[i].lastValidMs == 0 ||
        nowMs - zone[i].lastValidMs > Cfg::MAX_SENSOR_AGE_MS ||
        zone[i].validStreak < 3) {
      allFresh = false;
    }

    const float c = zoneConfidence(i, nowMs);

    if (c < minimumConfidence) minimumConfidence = c;
  }

  data.overallConfidence = allFresh ? minimumConfidence : 0.35f;
  data.frontConfidence = zoneConfidence(Pin::ZONE_FRONT, nowMs);

  const ZoneData &front = zone[Pin::ZONE_FRONT];

  const bool frontFresh =
      front.lastValidMs > 0 &&
      nowMs - front.lastValidMs <= Cfg::MAX_SENSOR_AGE_MS &&
      front.validStreak >= 3 &&
      front.latestSampleValid;

  data.frontRangeM =
      (frontFresh && isfinite(front.filteredCm))
          ? front.filteredCm / 100.0f
          : NAN;

  data.closingMps = frontFresh ? front.closingMps : 0.0f;

  data.frontTtcS =
      (frontFresh && data.closingMps > Cfg::MIN_CLOSING_SPEED_MPS)
          ? data.frontRangeM / data.closingMps
          : NAN;

  data.ttcS = data.frontTtcS;
}

void beginCalibration() {
  data.pathCandidate = false;
  data.pathConflict = false;
  data.pathCount = 0;
  data.pathExitCount = 0;
  data.hazardConfirmed = false;
  data.hazardConfidence = 0.0f;
  data.hazardZone = -1;

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    zone[i].pathCandidate = false;
    zone[i].pathConflict = false;
    zone[i].pathCount = 0;
    zone[i].pathExitCount = 0;
    zone[i].ttcS = NAN;
  }

  data.dropAlert = false;
  data.raisedAlert = false;
  data.indicator = STATE_LOW_CONFIDENCE;

  decisionCandidate = STATE_LOW_CONFIDENCE;
  decisionCandidateCount = 0;

  data.calibrationActive = true;
  data.baselineReady = false;

  calibrationCount = 0;
  calibrationSum = 0.0f;
  lastCalibrationSampleMs = 0;

  data.dropCount = 0;
  data.dropExitCount = 0;
  data.raisedCount = 0;
  data.raisedExitCount = 0;

  data.dropAlert = false;
  data.raisedAlert = false;

  Serial.println(
      "{\"event\":\"calibration_started\","
      "\"instruction\":\"hold normal-floor cane range/pitch\"}");
}

void updateCalibration(uint32_t nowMs) {
  if (!data.calibrationActive ||
      nowMs - lastCalibrationSampleMs < Cfg::CALIBRATION_SAMPLE_MS) {
    return;
  }

  lastCalibrationSampleMs = nowMs;

  const uint16_t rangeRaw =
      static_cast<uint16_t>(analogRead(Pin::POT_CANE_RANGE));

  const uint16_t pitchRaw =
      static_cast<uint16_t>(analogRead(Pin::POT_CANE_PITCH));

  calibrationSum += projectedHeight(
      rawToCaneRange(rangeRaw),
      rawToPitch(pitchRaw));

  ++calibrationCount;

  if (calibrationCount >= Cfg::CALIBRATION_SAMPLES) {
    data.baselineHeightCm =
        calibrationSum / static_cast<float>(calibrationCount);

    data.baselineReady = true;
    data.calibrationActive = false;

    Serial.print("{\"event\":\"baseline_calibrated\",\"baseline_h_cm\":");
    Serial.print(data.baselineHeightCm, 2);
    Serial.println("}");
  }
}

bool updatePotInputs(uint32_t nowMs) {
  if (nowMs - lastPotMs < Cfg::POT_INTERVAL_MS) return false;

  lastPotMs = nowMs;

  data.rawRange =
      static_cast<uint16_t>(analogRead(Pin::POT_CANE_RANGE));

  data.rawPitch =
      static_cast<uint16_t>(analogRead(Pin::POT_CANE_PITCH));

  data.rawLateral =
      static_cast<uint16_t>(analogRead(Pin::POT_LATERAL_OFFSET));

  const float range = rawToCaneRange(data.rawRange);
  const float pitch = rawToPitch(data.rawPitch);
  const float lateral = rawToLateral(data.rawLateral);

  data.caneRangeCm = ema(data.caneRangeCm, range, Cfg::POT_EMA_ALPHA);
  data.pitchDeg = ema(data.pitchDeg, pitch, Cfg::POT_EMA_ALPHA);
  data.lateralM = ema(data.lateralM, lateral, Cfg::POT_EMA_ALPHA);

  const float previousProjectedHeight = data.projectedHeightCm;

  data.projectedHeightCm =
      projectedHeight(data.caneRangeCm, data.pitchDeg);

  if (isfinite(previousProjectedHeight) &&
      previousProjectedHeight > 0.0f) {

    const float stepCm =
        fabsf(data.projectedHeightCm - previousProjectedHeight);

    data.caneStepEmaCm =
        ema(data.caneStepEmaCm, stepCm, Cfg::NOISE_EMA_ALPHA);

    data.caneConfidence =
        clampFloat(0.95f - 0.10f * data.caneStepEmaCm, 0.0f, 0.95f);
  } else {
    data.caneStepEmaCm = 0.0f;
    data.caneConfidence = 0.95f;
  }

  data.deltaHeightCm = data.baselineReady
                           ? data.projectedHeightCm - data.baselineHeightCm
                           : 0.0f;

  if (!data.baselineReady || data.calibrationActive) return true;

  // Moving/unstable test inputs cannot confirm or clear a drop-off proxy.
  if (data.caneConfidence < Cfg::MIN_CONFIDENCE) {
    data.dropCount = 0;
    data.raisedCount = 0;
    return true;
  }

  if (data.deltaHeightCm > Cfg::DROP_DELTA_CM) {
    if (data.dropCount < 250) ++data.dropCount;

    data.dropExitCount = 0;

    if (data.dropCount >= Cfg::CONFIRM_SAMPLES) {
      data.dropAlert = true;
    }
  } else {
    data.dropCount = 0;

    if (data.dropAlert &&
        data.deltaHeightCm < Cfg::DROP_CLEAR_DELTA_CM) {

      if (data.dropExitCount < 250) ++data.dropExitCount;

      if (data.dropExitCount >= Cfg::CONFIRM_SAMPLES) {
        data.dropAlert = false;
        data.dropExitCount = 0;
      }
    } else {
      data.dropExitCount = 0;
    }
  }

  if (data.deltaHeightCm < -Cfg::RAISED_DELTA_CM) {
    if (data.raisedCount < 250) ++data.raisedCount;

    data.raisedExitCount = 0;

    if (data.raisedCount >= Cfg::CONFIRM_SAMPLES) {
      data.raisedAlert = true;
    }
  } else {
    data.raisedCount = 0;

    if (data.raisedAlert &&
        data.deltaHeightCm > Cfg::RAISED_CLEAR_DELTA_CM) {

      if (data.raisedExitCount < 250) ++data.raisedExitCount;

      if (data.raisedExitCount >= Cfg::CONFIRM_SAMPLES) {
        data.raisedAlert = false;
        data.raisedExitCount = 0;
      }
    } else {
      data.raisedExitCount = 0;
    }
  }

  return true;
}

bool zoneIsFresh(uint8_t index, uint32_t nowMs) {
  return zone[index].lastValidMs > 0 &&
         nowMs - zone[index].lastValidMs <= Cfg::MAX_SENSOR_AGE_MS &&
         zone[index].validStreak >= 3 &&
         zone[index].latestSampleValid;
}

void updateZonePathConflict(uint8_t index, uint32_t nowMs) {
  ZoneData &z = zone[index];

  z.pathCandidate = false;
  z.ttcS = NAN;

  if (!zoneIsFresh(index, nowMs) || !isfinite(z.filteredCm)) {
    z.pathConflict = false;
    z.pathCount = 0;
    z.pathExitCount = 0;
    return;
  }

  bool lateralAligned = false;

  if (index == Pin::ZONE_LEFT) {
    lateralAligned = data.lateralM <= -Cfg::PATH_HALF_WIDTH_M;
  } else if (index == Pin::ZONE_FRONT) {
    lateralAligned = fabsf(data.lateralM) <= Cfg::PATH_HALF_WIDTH_M;
  } else if (index == Pin::ZONE_RIGHT) {
    lateralAligned = data.lateralM >= Cfg::PATH_HALF_WIDTH_M;
  }

  if (lateralAligned &&
      z.filteredCm <= Cfg::ZONE_NEAR_CM &&
      z.closingMps > Cfg::MIN_CLOSING_SPEED_MPS) {

    z.ttcS = (z.filteredCm / 100.0f) / z.closingMps;

    const float confidence = zoneConfidence(index, nowMs);

    z.pathCandidate =
        confidence >= Cfg::MIN_CONFIDENCE &&
        isfinite(z.ttcS) &&
        z.ttcS > 0.0f &&
        z.ttcS <= Cfg::TTC_WARNING_S;
  }

  if (z.pathCandidate) {
    if (z.pathCount < 250) ++z.pathCount;

    z.pathExitCount = 0;

    if (z.pathCount >= Cfg::CONFIRM_SAMPLES) {
      z.pathConflict = true;
    }
  } else {
    z.pathCount = 0;

    if (z.pathConflict) {
      if (z.pathExitCount < 250) ++z.pathExitCount;

      if (z.pathExitCount >= Cfg::CONFIRM_SAMPLES) {
        z.pathConflict = false;
        z.pathExitCount = 0;
      }
    } else {
      z.pathExitCount = 0;
    }
  }
}

void aggregatePathConflicts(uint32_t nowMs) {
  data.pathCandidate = false;
  data.pathConflict = false;
  data.pathCount = 0;
  data.pathExitCount = 0;
  data.hazardConfirmed = false;
  data.hazardConfidence = 0.0f;
  data.hazardZone = -1;
  data.ttcS = data.frontTtcS;

  float bestTtc = INFINITY;

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    ZoneData &z = zone[i];

    if (!zoneIsFresh(i, nowMs)) {
      z.pathCandidate = false;
      z.pathConflict = false;
      z.pathCount = 0;
      z.pathExitCount = 0;
      z.ttcS = NAN;
      continue;
    }

    if (z.pathCandidate) data.pathCandidate = true;

    if (z.pathCount > data.pathCount) {
      data.pathCount = z.pathCount;
    }

    if (z.pathConflict) {
      data.pathConflict = true;

      const float candidateTtc =
          isfinite(z.ttcS) ? z.ttcS : INFINITY;

      if (data.hazardZone < 0 || candidateTtc < bestTtc) {
        bestTtc = candidateTtc;
        data.hazardZone = static_cast<int8_t>(i);
        data.hazardConfidence = zoneConfidence(i, nowMs);

        if (isfinite(z.ttcS)) {
          data.ttcS = z.ttcS;
        }
      }
    }
  }

  if (data.dropAlert) {
    // The cane-pot drop-off proxy has priority and always uses the front sounder.
    data.hazardConfirmed = data.caneConfidence >= Cfg::MIN_CONFIDENCE;
    data.hazardConfidence = data.caneConfidence;
    data.hazardZone = static_cast<int8_t>(Pin::ZONE_FRONT);
    data.ttcS = NAN;
  } else {
    data.hazardConfirmed =
        data.pathConflict &&
        data.hazardConfidence >= Cfg::MIN_CONFIDENCE;

    if (!data.hazardConfirmed && !data.pathConflict) {
      data.hazardConfidence = 0.0f;
    }
  }
}

int8_t nearestConfirmedZone(uint32_t nowMs) {
  int8_t best = -1;
  float nearest = INFINITY;

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    if (!zoneIsFresh(i, nowMs) ||
        zone[i].nearStreak < Cfg::ZONE_CONFIRM_SAMPLES) {
      continue;
    }

    if (zone[i].filteredCm < nearest) {
      nearest = zone[i].filteredCm;
      best = static_cast<int8_t>(i);
    }
  }

  return best;
}

uint8_t requestedIndicator(uint32_t nowMs) {
  if (data.calibrationActive || !data.baselineReady) {
    return STATE_LOW_CONFIDENCE;
  }

  if (data.dropAlert || data.pathConflict) {
    return STATE_CRITICAL;
  }

  if (data.raisedAlert) {
    return STATE_RAISED;
  }

  if (data.overallConfidence < Cfg::MIN_CONFIDENCE ||
      !zoneIsFresh(Pin::ZONE_LEFT, nowMs) ||
      !zoneIsFresh(Pin::ZONE_FRONT, nowMs) ||
      !zoneIsFresh(Pin::ZONE_RIGHT, nowMs)) {
    return STATE_LOW_CONFIDENCE;
  }

  const int8_t selected = nearestConfirmedZone(nowMs);

  if (selected == static_cast<int8_t>(Pin::ZONE_LEFT)) {
    return STATE_LEFT;
  }

  if (selected == static_cast<int8_t>(Pin::ZONE_FRONT)) {
    return STATE_FRONT;
  }

  if (selected == static_cast<int8_t>(Pin::ZONE_RIGHT)) {
    return STATE_RIGHT;
  }

  return STATE_CLEAR;
}

void decideIndicator(uint32_t nowMs, bool freshObservation) {
  const uint8_t target = requestedIndicator(nowMs);

  // A confirmed critical state is applied immediately.
  // Other state changes need three fresh updates.
  if (target == STATE_CRITICAL) {
    data.indicator = STATE_CRITICAL;
    decisionCandidate = target;
    decisionCandidateCount = 0;
    return;
  }

  if (target == data.indicator) {
    decisionCandidate = target;
    decisionCandidateCount = 0;
    return;
  }

  if (!freshObservation) return;

  if (target != decisionCandidate) {
    decisionCandidate = target;
    decisionCandidateCount = 1;
  } else if (decisionCandidateCount < Cfg::CONFIRM_SAMPLES) {
    ++decisionCandidateCount;
  }

  if (decisionCandidateCount >= Cfg::CONFIRM_SAMPLES) {
    data.indicator = decisionCandidate;
    decisionCandidateCount = 0;
  }
}

void writeIndicator(uint8_t stateIndex) {
  if (stateIndex >= STATE_COUNT || stateIndex == lastWrittenLedState) {
    return;
  }

  const uint8_t oneHot = static_cast<uint8_t>(1U << stateIndex);

  digitalWrite(Pin::LED_LATCH, LOW);

  shiftOut(
      Pin::LED_DATA,
      Pin::LED_CLOCK,
      MSBFIRST,
      oneHot);

  digitalWrite(Pin::LED_LATCH, HIGH);

  lastWrittenLedState = stateIndex;
}

uint8_t buzzerPinForState(uint8_t stateIndex) {
  switch (stateIndex) {
    case STATE_CRITICAL:
      return Pin::BUZZER_FRONT;

    case STATE_LEFT:
      return Pin::BUZZER_LEFT;

    case STATE_FRONT:
      return Pin::BUZZER_FRONT;

    case STATE_RIGHT:
      return Pin::BUZZER_RIGHT;

    default:
      return NO_PIN;
  }
}

uint8_t patternLengthForState(uint8_t stateIndex) {
  switch (stateIndex) {
    case STATE_CRITICAL:
      return 6;

    case STATE_LEFT:
      return 2;

    case STATE_FRONT:
      return 4;

    case STATE_RIGHT:
      return 6;

    default:
      return 0;
  }
}

// Primitive arguments only: safe from Arduino .ino custom-type prototype ordering.
void getPatternPhase(
    uint8_t stateIndex,
    uint8_t phaseIndex,
    uint16_t &frequencyHz,
    uint16_t &durationMs) {

  frequencyHz = 0;
  durationMs = 900;

  if (stateIndex == STATE_CRITICAL) {
    const uint16_t f[6] = {2350, 0, 2350, 0, 2350, 0};
    const uint16_t d[6] = {150, 90, 150, 90, 220, 650};

    if (phaseIndex < 6) {
      frequencyHz = f[phaseIndex];
      durationMs = d[phaseIndex];
    }
  } else if (stateIndex == STATE_LEFT) {
    if (phaseIndex == 0) {
      frequencyHz = 1050;
      durationMs = 120;
    } else {
      frequencyHz = 0;
      durationMs = 1100;
    }
  } else if (stateIndex == STATE_FRONT) {
    const uint16_t f[4] = {1500, 0, 1500, 0};
    const uint16_t d[4] = {100, 110, 100, 950};

    if (phaseIndex < 4) {
      frequencyHz = f[phaseIndex];
      durationMs = d[phaseIndex];
    }
  } else if (stateIndex == STATE_RIGHT) {
    const uint16_t f[6] = {1950, 0, 1950, 0, 1950, 0};
    const uint16_t d[6] = {80, 90, 80, 90, 80, 900};

    if (phaseIndex < 6) {
      frequencyHz = f[phaseIndex];
      durationMs = d[phaseIndex];
    }
  }
}

int8_t requestedAudioState(uint32_t nowMs) {
  if (data.calibrationActive ||
      !data.baselineReady ||
      !data.hazardConfirmed ||
      data.hazardConfidence < Cfg::MIN_CONFIDENCE) {
    return static_cast<int8_t>(STATE_CLEAR);
  }

  if (data.dropAlert) {
    return static_cast<int8_t>(STATE_CRITICAL);
  }

  if (data.pathConflict &&
      data.hazardZone >= 0 &&
      zoneIsFresh(static_cast<uint8_t>(data.hazardZone), nowMs)) {

    if (data.hazardZone == static_cast<int8_t>(Pin::ZONE_LEFT)) {
      return static_cast<int8_t>(STATE_LEFT);
    }

    if (data.hazardZone == static_cast<int8_t>(Pin::ZONE_FRONT)) {
      return static_cast<int8_t>(STATE_FRONT);
    }

    if (data.hazardZone == static_cast<int8_t>(Pin::ZONE_RIGHT)) {
      return static_cast<int8_t>(STATE_RIGHT);
    }
  }

  return static_cast<int8_t>(STATE_CLEAR);
}

void stopActiveBuzzer() {
  // Stop only the channel this state machine started.
  if (audio.soundingPin >= 0) {
    noTone(static_cast<uint8_t>(audio.soundingPin));
    audio.soundingPin = -1;
  }

  audio.toneOn = false;
}

void applyAudioPhase(uint32_t nowMs) {
  const uint8_t count =
      patternLengthForState(static_cast<uint8_t>(audio.activeState));

  if (audio.phase >= count) {
    stopActiveBuzzer();
    audio.phase = 0;
  }

  uint16_t frequencyHz = 0;
  uint16_t durationMs = 0;

  getPatternPhase(
      static_cast<uint8_t>(audio.activeState),
      audio.phase,
      frequencyHz,
      durationMs);

  audio.phaseStartedMs = nowMs;

  (void)durationMs;

  stopActiveBuzzer();

  const uint8_t pin =
      buzzerPinForState(static_cast<uint8_t>(audio.activeState));

  if (frequencyHz > 0 && pin != NO_PIN) {
    tone(pin, frequencyHz);
    audio.soundingPin = static_cast<int8_t>(pin);
    audio.toneOn = true;
  }
}

void updateAudio(uint32_t nowMs) {
  // Audio is enabled only for confirmed, high-confidence hazards.
  const int8_t requested = requestedAudioState(nowMs);

  if (requested != audio.activeState) {
    stopActiveBuzzer();

    audio.activeState = requested;
    audio.phase = 0;
    audio.phaseStartedMs = nowMs;

    if (requested == static_cast<int8_t>(STATE_CLEAR)) {
      return;
    }

    applyAudioPhase(nowMs);
    return;
  }

  if (audio.activeState < 0 ||
      audio.activeState == static_cast<int8_t>(STATE_CLEAR)) {
    return;
  }

  uint16_t frequencyHz = 0;
  uint16_t durationMs = 0;

  getPatternPhase(
      static_cast<uint8_t>(audio.activeState),
      audio.phase,
      frequencyHz,
      durationMs);

  (void)frequencyHz;

  if (nowMs - audio.phaseStartedMs < durationMs) {
    return;
  }

  ++audio.phase;

  const uint8_t count =
      patternLengthForState(static_cast<uint8_t>(audio.activeState));

  if (audio.phase >= count) {
    audio.phase = 0;
  }

  applyAudioPhase(nowMs);
}

void printNullableFloat(float value, uint8_t decimals) {
  if (isfinite(value)) {
    Serial.print(value, decimals);
  } else {
    Serial.print("null");
  }
}

void printJson(uint32_t nowMs) {
  if (nowMs - lastLogMs < Cfg::LOG_INTERVAL_MS) return;

  lastLogMs = nowMs;

  Serial.print("{\"ts_ms\":");
  Serial.print(nowMs);

  Serial.print(",\"mode\":\"WOKWI_SINGLE_NODE_SIM\"");

  Serial.print(",\"cane_range_cm_proxy\":");
  Serial.print(data.caneRangeCm, 2);

  Serial.print(",\"cane_pitch_deg_proxy\":");
  Serial.print(data.pitchDeg, 2);

  Serial.print(",\"cane_confidence_heuristic\":");
  Serial.print(data.caneConfidence, 2);

  Serial.print(",\"lateral_offset_m_proxy\":");
  Serial.print(data.lateralM, 2);

  Serial.print(",\"projected_h_cm\":");
  Serial.print(data.projectedHeightCm, 2);

  Serial.print(",\"baseline_h_cm\":");
  printNullableFloat(data.baselineHeightCm, 2);

  Serial.print(",\"delta_h_cm\":");
  Serial.print(data.deltaHeightCm, 2);

  Serial.print(",\"dropoff_confirm_count\":");
  Serial.print(data.dropCount);

  Serial.print(",\"dropoff\":");
  Serial.print(data.dropAlert ? "true" : "false");

  Serial.print(",\"raised\":");
  Serial.print(data.raisedAlert ? "true" : "false");

  Serial.print(",\"left_cm\":");
  printNullableFloat(zone[Pin::ZONE_LEFT].filteredCm, 1);

  Serial.print(",\"front_cm\":");
  printNullableFloat(zone[Pin::ZONE_FRONT].filteredCm, 1);

  Serial.print(",\"right_cm\":");
  printNullableFloat(zone[Pin::ZONE_RIGHT].filteredCm, 1);

  Serial.print(",\"front_closing_mps_heuristic\":");
  printNullableFloat(data.closingMps, 2);

  Serial.print(",\"ttc_s_heuristic\":");
  printNullableFloat(data.ttcS, 2);

  Serial.print(",\"front_confidence_heuristic\":");
  Serial.print(data.frontConfidence, 2);

  Serial.print(",\"confidence_heuristic\":");
  Serial.print(data.overallConfidence, 2);

  Serial.print(",\"path_conflict_candidate\":");
  Serial.print(data.pathCandidate ? "true" : "false");

  Serial.print(",\"path_confirm_count\":");
  Serial.print(data.pathCount);

  Serial.print(",\"path_conflict\":");
  Serial.print(data.pathConflict ? "true" : "false");

  Serial.print(",\"hazard_confirmed\":");
  Serial.print(data.hazardConfirmed ? "true" : "false");

  Serial.print(",\"hazard_confidence_heuristic\":");
  Serial.print(data.hazardConfidence, 2);

  Serial.print(",\"hazard_zone\":");

  if (data.hazardZone < 0) {
    Serial.print("null");
  } else {
    Serial.print(ZONE_NAMES[static_cast<uint8_t>(data.hazardZone)]);
  }

  Serial.print(",\"indicator\":\"");
  Serial.print(STATE_NAMES[data.indicator]);
  Serial.print("\"");

  Serial.print(",\"buzzer_pin\":");

  const uint8_t pin =
      (audio.activeState < 0 ||
       audio.activeState == static_cast<int8_t>(STATE_CLEAR))
          ? NO_PIN
          : buzzerPinForState(static_cast<uint8_t>(audio.activeState));

  if (pin == NO_PIN) {
    Serial.print("null");
  } else {
    Serial.print(pin);
  }

  Serial.print(",\"buzzer_tone_on\":");
  Serial.print(audio.toneOn ? "true" : "false");

  Serial.print(",\"valid_zone_count\":");

  uint8_t validCount = 0;

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    if (zoneIsFresh(i, nowMs)) ++validCount;
  }

  Serial.print(validCount);

  Serial.print(",\"baseline_ready\":");
  Serial.print(data.baselineReady ? "true" : "false");

  Serial.print(",\"mpu_ok\":");
  Serial.print(data.mpuReadingValid ? "true" : "false");

  Serial.print(",\"accel_g\":[");

  if (data.mpuReadingValid) {
    Serial.print(data.accelX, 3);
    Serial.print(',');

    Serial.print(data.accelY, 3);
    Serial.print(',');

    Serial.print(data.accelZ, 3);
  } else {
    Serial.print("null,null,null");
  }

  Serial.println("]}");
}

void printHelpJson() {
  Serial.println(
      "{\"event\":\"help\","
      "\"commands\":[\"c\",\"?\"],"
      "\"c\":\"start non-blocking baseline calibration; hold cane range/pitch at normal-floor values\","
      "\"?\":\"show help\"}");

  Serial.println(
      "{\"event\":\"logic\","
      "\"path_conflict\":\"matching left/front/right lateral sector, confidence >=0.60, closing speed >0.08m/s, TTC <=3s, and 3 confirmed samples\","
      "\"dropoff\":\"projected-height proxy delta >15cm, stable cane input, 3 confirmations after baseline calibration\","
      "\"audio\":\"only confirmed high-confidence drop-off/path conflicts; proximity/raised/low-confidence are visual-only\","
      "\"wiring\":\"LED/resistor branches and buzzer branches are electrically independent\"}");
}

void processSerialLine() {
  serialLine[serialLineLength] = '\0';

  if (strcmp(serialLine, "c") == 0 ||
      strcmp(serialLine, "C") == 0) {
    beginCalibration();
  } else if (strcmp(serialLine, "?") == 0 ||
             strcmp(serialLine, "help") == 0) {
    printHelpJson();
  } else if (serialLineLength > 0) {
    Serial.println(
        "{\"event\":\"command_error\","
        "\"message\":\"unknown command; use c or ?\"}");
  }

  serialLineLength = 0;
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      if (serialLineLength > 0) {
        processSerialLine();
      }
    } else if (static_cast<size_t>(serialLineLength) + 1U <
               sizeof(serialLine)) {
      serialLine[serialLineLength++] = c;
    } else {
      serialLineLength = 0;

      Serial.println(
          "{\"event\":\"command_error\","
          "\"message\":\"line too long\"}");
    }
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);

  pinMode(Pin::POT_CANE_RANGE, INPUT);
  pinMode(Pin::POT_CANE_PITCH, INPUT);
  pinMode(Pin::POT_LATERAL_OFFSET, INPUT);

  pinMode(Pin::LED_DATA, OUTPUT);
  pinMode(Pin::LED_CLOCK, OUTPUT);
  pinMode(Pin::LED_LATCH, OUTPUT);

  digitalWrite(Pin::LED_DATA, LOW);
  digitalWrite(Pin::LED_CLOCK, LOW);
  digitalWrite(Pin::LED_LATCH, LOW);

  for (uint8_t i = 0; i < Pin::ZONE_COUNT; ++i) {
    pinMode(Pin::TRIG[i], OUTPUT);
    pinMode(Pin::ECHO[i], INPUT);

    digitalWrite(Pin::TRIG[i], LOW);

    zone[i].filteredCm = NAN;
    zone[i].closingMps = 0.0f;
    zone[i].noiseEmaCm = 0.0f;

    zone[i].historyCount = 0;
    zone[i].historyNext = 0;

    zone[i].validStreak = 0;
    zone[i].nearStreak = 0;

    zone[i].previousSampleMs = 0;
    zone[i].lastValidMs = 0;

    zone[i].latestSampleValid = false;

    zone[i].pathCount = 0;
    zone[i].pathExitCount = 0;

    zone[i].pathCandidate = false;
    zone[i].pathConflict = false;

    zone[i].ttcS = NAN;

    for (uint8_t j = 0; j < 3; ++j) {
      zone[i].history[j] = NAN;
    }
  }

  pinMode(Pin::BUZZER_LEFT, OUTPUT);
  pinMode(Pin::BUZZER_FRONT, OUTPUT);
  pinMode(Pin::BUZZER_RIGHT, OUTPUT);

  digitalWrite(Pin::BUZZER_LEFT, LOW);
  digitalWrite(Pin::BUZZER_FRONT, LOW);
  digitalWrite(Pin::BUZZER_RIGHT, LOW);

  data.caneRangeCm = 40.0f;
  data.pitchDeg = 0.0f;
  data.lateralM = 0.0f;

  data.projectedHeightCm = 0.0f;
  data.caneStepEmaCm = 0.0f;
  data.caneConfidence = 0.95f;

  data.baselineHeightCm = NAN;
  data.deltaHeightCm = 0.0f;

  data.frontRangeM = NAN;
  data.closingMps = 0.0f;
  data.ttcS = NAN;
  data.frontTtcS = NAN;

  data.frontConfidence = 0.35f;
  data.overallConfidence = 0.35f;

  data.accelX = 0.0f;
  data.accelY = 0.0f;
  data.accelZ = 1.0f;

  data.rawRange = 0;
  data.rawPitch = 0;
  data.rawLateral = 0;

  data.dropCount = 0;
  data.dropExitCount = 0;

  data.raisedCount = 0;
  data.raisedExitCount = 0;

  data.pathCount = 0;
  data.pathExitCount = 0;

  data.baselineReady = false;
  data.calibrationActive = false;

  data.mpuReady = false;
  data.mpuReadingValid = false;

  data.dropAlert = false;
  data.raisedAlert = false;

  data.pathCandidate = false;
  data.pathConflict = false;

  data.hazardConfirmed = false;
  data.hazardConfidence = 0.0f;
  data.hazardZone = -1;

  data.indicator = STATE_LOW_CONFIDENCE;

  Wire.begin(Pin::MPU_SDA, Pin::MPU_SCL);
  Wire.setClock(100000);

  data.mpuReady = beginMpu6050();

  writeIndicator(STATE_LOW_CONFIDENCE);
  stopActiveBuzzer();

  // Asynchronous startup calibration.
  beginCalibration();

  Serial.print(
      "{\"event\":\"boot\","
      "\"project\":\"DualZone clean simulation\","
      "\"mode\":\"single-ESP32 Wokwi harness\","
      "\"wireless_link\":false,"
      "\"pot_count\":3,"
      "\"led_count\":7,"
      "\"buzzer_count\":3,"
      "\"mpu6050_detected\":");

  Serial.print(data.mpuReady ? "true" : "false");
  Serial.println("}");

  if (!data.mpuReady) {
    Serial.println(
        "{\"event\":\"sensor_warning\","
        "\"sensor\":\"MPU6050\","
        "\"status\":\"not detected; pot simulation remains available\"}");
  }

  Serial.println(
      "{\"event\":\"layout\","
      "\"sections\":[\"CANE INPUTS\",\"ESP32 PROCESSOR\",\"BELT SENSORS + INDICATORS\"]}");

  printHelpJson();
}

void loop() {
  pollSerialCommands();

  uint32_t nowMs = millis();
  bool freshObservation = false;

  updateCalibration(nowMs);

  if (updatePotInputs(nowMs)) {
    freshObservation = true;
  }

  // Only one ultrasound measurement per slot.
  // The three sensors are never triggered together.
  int8_t scannedZone = -1;

  if (nowMs - lastScanMs >= Cfg::ZONE_SCAN_INTERVAL_MS) {
    lastScanMs = nowMs;

    const uint8_t index = static_cast<uint8_t>(nextZoneToScan);
    const float cm = readDistanceCm(index);

    nowMs = millis();

    updateZone(index, cm, nowMs);

    scannedZone = static_cast<int8_t>(index);

    nextZoneToScan =
        (nextZoneToScan + 1U) % Pin::ZONE_COUNT;

    freshObservation = true;
  }

  nowMs = millis();

  updateCalibration(nowMs);
  updateOverallConfidence(nowMs);

  if (scannedZone >= 0) {
    updateZonePathConflict(
        static_cast<uint8_t>(scannedZone),
        nowMs);
  }

  aggregatePathConflicts(nowMs);

  if (nowMs - lastImuMs >= Cfg::IMU_INTERVAL_MS) {
    lastImuMs = nowMs;
    data.mpuReadingValid =
        data.mpuReady && readMpuAcceleration();
  }

  decideIndicator(nowMs, freshObservation);

  writeIndicator(data.indicator);

  updateAudio(nowMs);

  printJson(nowMs);

  pollSerialCommands();
}

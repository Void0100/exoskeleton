#include "hx711.h"
#include "state.h"

const char* hxPhase = "BOOT";

// Diagnostics only; no timeout/threshold/control effect.

// Per-side sample interval statistics.
static uint32_t leftSum = 0, leftCount = 0, leftMin = UINT32_MAX, leftMax = 0;
static uint32_t rightSum = 0, rightCount = 0, rightMin = UINT32_MAX, rightMax = 0;
static uint32_t indepRateLastPrint = 0;

static const char* pinLevelName(int level) { return level ? "HIGH" : "LOW"; }

static void printLastSample(const char* sideName, bool hasSample,
  int32_t rawCounts, uint32_t sampleMs, uint32_t nowMs) {
  if (hasSample) {
    Serial.printf("    %s: %ld counts, %lu ms ago\n",
      sideName, (long)rawCounts, (unsigned long)(nowMs - sampleMs));
  } else {
    Serial.printf("    %s: none yet\n", sideName);
  }
}

static void printHxError(const char* reason, uint32_t waitMs) {
  // uint32_t nowMs = millis();

  int leftDataLevel = digitalRead(LEFT_FORCE_DATA_PIN);
  int rightDataLevel = digitalRead(RIGHT_FORCE_DATA_PIN);
  // int leftClockLevel = digitalRead(LEFT_FORCE_CLOCK_PIN);
  // int rightClockLevel = digitalRead(RIGHT_FORCE_CLOCK_PIN);

  // bool leftReady = (leftDataLevel == LOW); // HX711 pulls data LOW when a sample is ready
  // bool rightReady = (rightDataLevel == LOW);

  log_e("HX711 TIMEOUT (phase: %s)\n \
    cause : %s after %lu ms(limit % lu ms)\n ",
    hxPhase, reason, (unsigned long)waitMs, (unsigned long)HX_TIMEOUT_MS
  );

  log_d(
    "  left  : data GPIO%d = %s (%s), clock GPIO%d = %s\n"
    "  right : data GPIO%d = %s (%s), clock GPIO%d = %s\n",
    LEFT_FORCE_DATA_PIN, pinLevelName(leftDataLevel), leftReady ? "ready" : "not ready", LEFT_FORCE_CLOCK_PIN,
    pinLevelName(leftClockLevel)
  );

  // Serial.println("  last good samples (raw forces):");
  // printLastSample("left ", haveLeftSample, leftLatestForceRaw, leftForceLatestMs, nowMs);
  // printLastSample("right", haveRightSample, rightLatestForceRaw, rightForceLatestMs, nowMs);
}

// static void updateHxRate(bool leftSide, uint32_t now) {
//   uint32_t& prev = leftSide ? leftPrevMs : rightPrevMs;
//   uint32_t& sum = leftSide ? leftSum : rightSum;
//   uint32_t& cnt = leftSide ? leftCount : rightCount;
//   uint32_t& mn = leftSide ? leftMin : rightMin;
//   uint32_t& mx = leftSide ? leftMax : rightMax;
//   if (prev) {
//     uint32_t dt = now - prev;
//     if (dt && dt < 1000) {
//       sum += dt;
//       cnt++;
//       if (dt < mn) mn = dt;
//       if (dt > mx) mx = dt;
//     }
//   }
//   prev = now;
// }

ForceSensor::ForceSensor(int dataPin, int clockPin) {
    dataPin = dataPin;
    clockPin = clockPin;
    prevForceRawMs = 0;
    hasFirstSample = false;
}

// statics
/** not done */
double ForceSensor::rawToNewton(int32_t raw) {
  return (double) raw;
}
        
// helpers
bool ForceSensor::isFresh() {
  return millis() - prevForceRawMs > HX_MAX_STALE_MS;
}

// blocking function
bool ForceSensor::readForceRaw(int32_t& output) {
  if (digitalRead(dataPin) != LOW) return false; // low == data pin ready
  uint32_t raw = 0;
  noInterrupts();
  for (int i = 0; i < 24; i++) {
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    raw = (raw << 1) | (digitalRead(dataPin) ? 1U : 0U);
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(clockPin, HIGH);  // 25th pulse: channel A, gain 128
  delayMicroseconds(1);
  digitalWrite(clockPin, LOW);
  interrupts();
  if (raw & 0x800000UL) raw |= 0xFF000000UL; // ready for turning it back to signed again
  output = (int32_t)raw;

  // update static variables
  prevForceRaw = raw;
  prevForceRawMs = millis();
  hasFirstSample = true;

  return true;
}

bool ForceSensor::readForce(double& output) {
  int32_t raw = 0;
  bool succ = readForceRaw(raw);
  if (!succ) {
    log_e("readForceRaw failed.");
    return false;
  }
  return rawToNewton(raw);
}

// non-blocking function
void ForceSensor::serviceUpdateForce() {
  if (digitalRead(dataPin) == LOW) {
    int32_t raw = 0;
    readForceRaw(raw);
    // int32_t raw = 0;
    // if (readForceRaw(raw)) {
    //   prevRawForce = raw;
    //   prevRawForceMs = millis();
    //   hasFirstSample = true;
    //   // updateHxRate(true, leftPrevMs);
    // }
  }
  // uint32_t now = millis();
  // if (now - indepRateLastPrint >= 2000) {
  //   indepRateLastPrint = now;
  //   double la = leftCount ? (double)leftSum / leftCount : 0.0;
  //   double ra = rightCount ? (double)rightSum / rightCount : 0.0;

  //   log_d(
  //     "HX_INDEP_RATE L=%.1fms %.1fSPS min=%lums max=%lums | R=%.1fms %.1fSPS min=%lums max=%lums\n",
  //     la, la ? 1000.0 / la : 0.0, (unsigned long)(leftCount ? leftMin : 0), (unsigned long)leftMax,
  //     ra, ra ? 1000.0 / ra : 0.0, (unsigned long)(rightCount ? rightMin : 0), (unsigned long)rightMax
  //   );

  //   leftSum = leftCount = 0;
  //   leftMin = UINT32_MAX;
  //   leftMax = 0;
  //   rightSum = rightCount = 0;
  //   rightMin = UINT32_MAX;
  //   rightMax = 0;
  // }
}

bool ForceSensor::getPrevForceRaw(int32_t& output) {
  if (!isFresh()) {
    log_e("The prev is too old, call serviceUpdateForce before reading again");
    return false;
  }
  return prevForceRaw;
}

bool ForceSensor::getPrevForce(double& output) {
  int32_t raw = 0;
  bool succ = getPrevForceRaw(raw);
  if (!succ) {
    log_e("getPrevForce failed.");
    return false;
  }
  return rawToNewton(raw);
}

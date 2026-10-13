#include "hx711.h"

const char *hxPhase = "BOOT";
bool leftFresh = false, rightFresh = false;
int32_t leftLatestForceRaw = 0, rightLatestForceRaw = 0;
uint32_t leftForceLatestMs = 0, rightForceLatestMs = 0;

// Diagnostics only; no timeout/threshold/control effect.
static bool haveLeftSample = false, haveRightSample = false;

// Per-side sample interval statistics.
static uint32_t leftPrevMs = 0, rightPrevMs = 0;
static uint32_t leftSum = 0, leftCount = 0, leftMin = UINT32_MAX, leftMax = 0;
static uint32_t rightSum = 0, rightCount = 0, rightMin = UINT32_MAX, rightMax = 0;
static uint32_t indepRateLastPrint = 0;

static const char *pinLevelName(int level) { return level ? "HIGH" : "LOW"; }

static void printLastSample(const char *sideName, bool hasSample,
                            int32_t rawCounts, uint32_t sampleMs, uint32_t nowMs) {
  if (hasSample) {
    Serial.printf("    %s: %ld counts, %lu ms ago\n",
                  sideName, (long)rawCounts, (unsigned long)(nowMs - sampleMs));
  } else {
    Serial.printf("    %s: none yet\n", sideName);
  }
}

void printHxError(const char *reason, uint32_t waitMs)
{
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

bool hx711ReadForceRaw(int dataPin, int clockPin, int32_t &output) {
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
  return true;
}

static void noteHxRate(bool leftSide, uint32_t now) {
  uint32_t &prev = leftSide ? leftPrevMs : rightPrevMs;
  uint32_t &sum = leftSide ? leftSum : rightSum;
  uint32_t &cnt = leftSide ? leftCount : rightCount;
  uint32_t &mn = leftSide ? leftMin : rightMin;
  uint32_t &mx = leftSide ? leftMax : rightMax;
  if (prev) {
    uint32_t dt = now - prev;
    if (dt && dt < 1000) {
      sum += dt;
      cnt++;
      if (dt < mn) mn = dt;
      if (dt > mx) mx = dt;
    }
  }
  prev = now;
}

void serviceIndependentHX() {
  if (digitalRead(LEFT_FORCE_DATA_PIN) == LOW) {
    int32_t v = 0;
    if (hx711ReadForceRaw(LEFT_FORCE_DATA_PIN, LEFT_FORCE_CLOCK_PIN, v))
    {
      leftFresh = true;
      leftLatestForceRaw = v;
      leftForceLatestMs = millis();
      haveLeftSample = true;
      noteHxRate(true, leftForceLatestMs);
    }
  }
  if (digitalRead(RIGHT_FORCE_DATA_PIN) == LOW) {
    int32_t v = 0;
    if (hx711ReadForceRaw(RIGHT_FORCE_DATA_PIN, RIGHT_FORCE_CLOCK_PIN, v)) {
      rightFresh = true;
      rightForceLatestMs = millis();
      rightLatestForceRaw = v;
      haveRightSample = true;
      noteHxRate(false, rightForceLatestMs);
    }
  }
  uint32_t now = millis();
  if (now - indepRateLastPrint >= 2000) {
    indepRateLastPrint = now;
    double la = leftCount ? (double)leftSum / leftCount : 0.0;
    double ra = rightCount ? (double)rightSum / rightCount : 0.0;

    log_i(
      "HX_INDEP_RATE L=%.1fms %.1fSPS min=%lums max=%lums | R=%.1fms %.1fSPS min=%lums max=%lums\n",
      la, la ? 1000.0 / la : 0.0, (unsigned long)(leftCount ? leftMin : 0), (unsigned long)leftMax,
      ra, ra ? 1000.0 / ra : 0.0, (unsigned long)(rightCount ? rightMin : 0), (unsigned long)rightMax
    );

    leftSum = leftCount = 0;
    leftMin = UINT32_MAX;
    leftMax = 0;
    rightSum = rightCount = 0;
    rightMin = UINT32_MAX;
    rightMax = 0;
  }
}

bool readForceLeft(int32_t &output) {
  uint32_t t = millis();
  while (digitalRead(LEFT_FORCE_DATA_PIN) != LOW) {
    if ((uint32_t)(millis() - t) >= HX_TIMEOUT_MS) {
      printHxError("LEFT_NOT_READY", millis() - t);
      return false;
    }
    yield();
  }

  int32_t forceRawLeft = 0;
  bool leftOK = hx711ReadForceRaw(LEFT_FORCE_DATA_PIN, LEFT_FORCE_CLOCK_PIN, forceRawLeft);
  if (leftOK) {
    leftLatestForceRaw = forceRawLeft;
    leftForceLatestMs = millis();
    haveLeftSample = true;
    return true;
  }
  printHxError("LEFT_READ_FAILED", millis() - t);
  return false;
}

bool readForceRight(int32_t& output) {
  uint32_t t = millis();
  while (digitalRead(RIGHT_FORCE_DATA_PIN) != LOW) {
    if ((uint32_t)(millis() - t) >= HX_TIMEOUT_MS) {
      printHxError("RIGHT_NOT_READY", millis() - t);
      return false;
    }
    yield();
  }

  int32_t forceRawRight = 0;
  bool success = hx711ReadForceRaw(RIGHT_FORCE_DATA_PIN, RIGHT_FORCE_CLOCK_PIN, forceRawRight);
  if (success) {
    rightLatestForceRaw = forceRawRight;
    rightForceLatestMs = millis();
    haveRightSample = true;
    return true;
  }
  printHxError("RIGHT_READ_FAILED", millis() - t);
  return false;
}

bool readForceBoth(int32_t &l, int32_t &r) {
  int32_t rightForceRaw = 0;
  int32_t leftForceRaw = 0;
  bool rightSuccess = readForceRight(rightForceRaw);
  bool leftSuccess = readForceLeft(leftForceRaw);

  if (!(rightSuccess && leftSuccess)) {
    log_e("Failed to measure forces.");
    return false;
  }
  return true;
}

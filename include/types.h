#pragma once
#include <Arduino.h>
#include <stdint.h>

// Reply of motor command 0x9C.
struct MotorStatus {
  int8_t temperature;
  int16_t iq, speed;
  uint16_t encoder;
};

// All per-motor runtime state. Independent axes: no opposing-intent gate.
struct Axis {
  uint8_t id;
  int anatomicalToRaw;      // +1 left, -1 right
  double home, base;        // encoder home (raw deg), HX baseline (counts)
  bool moving, released;
  int intent;               // -1, 0, +1
  uint8_t candidateCount;
  double target, start, lastAngle;
  uint32_t speed, began, updated;
  uint8_t overIq;
};

// One non-worn bench STOP characterization record (+50/+100/+200/+500 ms).
struct StopChar {
  bool active = false;
  uint8_t id = 0;
  int anatomicalToRaw = 1;
  uint32_t t0 = 0;
  double pos0 = NAN;
  int16_t speed0 = 0;
  bool got50 = false, got100 = false, got200 = false, got500 = false;
  double p50 = NAN, p100 = NAN, p200 = NAN, p500 = NAN;
  uint32_t settledAt = 0;
  double settledPos = NAN;
  uint8_t stableCount = 0;
};

#pragma once
#include <Arduino.h>
#include <stdint.h>

// Reply of motor command 0x9C.
struct MotorStatus {
  int8_t temperature;
  int16_t iq, speed;
  uint16_t encoder;
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

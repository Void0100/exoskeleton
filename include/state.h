#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

// Safety state machine flags. latched is permanent until reset.
extern bool stateArmed, stateLatched, stateCalibrated, stateEnabled;
// Captured by tare(): encoder home (raw deg) and HX baseline (counts).
extern double baseAngRawL, baseAngRawR, baseForceRawL, baseForceRawR;
// Selected index into TEST_SPEED_LEVEL_DPS (change only while disarmed).
extern uint8_t testSpeedLevel;
// Time the current force intent first appeared (0 = none).
extern uint32_t intentSinceL, intentSinceR;
extern Axis leftAxis, rightAxis;


// ---- API ----------------------------------------------------------------
/** Disarm, set latched, print reason, STOP/OFF both motors. Reset required. */
void latch(const char *reason);

void resetAllStates();
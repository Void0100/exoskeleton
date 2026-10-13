#pragma once
#include <Arduino.h>
#include "config.h"

// Phase label printed in HX_TIMEOUT diagnostics (set by callers).
extern const char *hxPhase;
// Newest sample per side; *Fresh = not yet consumed by the control loop.
extern bool leftFresh, rightFresh;
extern int32_t leftLatestForceRaw, rightLatestForceRaw;
extern uint32_t leftForceLatestMs, rightForceLatestMs;

bool hx711ReadForceRaw(int dt, int clk, int32_t &out);   // one 24-bit read; false if DOUT not ready
void printHxError(const char *reason, uint32_t waitMs);
void serviceIndependentHX();                 // non-blocking, each side on its own DOUT-ready
bool readForceBoth(int32_t &l, int32_t &r);         // blocking paired read (tare and P only)

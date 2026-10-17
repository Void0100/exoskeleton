#pragma once
#include <Arduino.h>
#include "config.h"

// Phase label printed in HX_TIMEOUT diagnostics (set by callers).
extern const char *hxPhase;

void serviceUpdateForces();                 // non-blocking, each side on its own DOUT-ready
bool serviceReadForceLeft(double& output);
bool serviceReadForceRight(double& output);
bool serviceReadForceBoth(double& l, double& r);

bool readForceLeft(double& output);
bool readForceRight(double& output);
bool readForceBoth(double &l, double &r);         // blocking paired read (tare and P only)


#pragma once
#include "types.h"

extern StopChar stopL, stopR;

StopChar &stopCharFor(uint8_t id);
void beginStopChar(const Axis &a, double posBefore, int16_t speedBefore);
void serviceStopCharOne(StopChar &q, const Axis &a);
void serviceStopChars();   // call every loop

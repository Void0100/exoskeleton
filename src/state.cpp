#include "state.h"
#include "rs485.h"

bool stateArmed = false, stateConfirmed = false, stateLatched = false, stateZeroed = false, stateEnabled = false;
double baseAngRawL = 0, baseAngRawR = 0, baseForceRawL = 0, baseForceRawR = 0;
uint8_t testSpeedLevel = 0;
uint32_t intentSinceL = 0, intentSinceR = 0;

Axis leftAxis{LEFT_BUS_ID, +1, 0, 0, false, true, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Axis rightAxis{RIGHT_BUS_ID, -1, 0, 0, false, true, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void resetAllStates() {
  stateArmed = false, stateConfirmed = false, stateLatched = false, stateZeroed = false, stateEnabled = false;
  baseAngRawL = 0, baseAngRawR = 0, baseForceRawL = 0, baseForceRawR = 0;
  testSpeedLevel = 0;
  intentSinceL = 0, intentSinceR = 0;
}


void latch(const char *reason) {
  stateArmed = false;
  stateLatched = true;
  Serial.printf("LATCHED: %s; use independent physical cutoff if unsafe. Reset required.\n", reason);
  rsStopBothMotors();
}

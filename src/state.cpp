#include "state.h"
#include "rs485.h"

bool armed = false, confirmed = false, latched = false, zeroed = false, enabled = false;
double homeL = 0, homeR = 0, baseL = 0, baseR = 0;
uint8_t testSpeedLevel = 0;
uint32_t intentSinceL = 0, intentSinceR = 0;

Axis leftAxis{LEFT_BUS_ID, +1, 0, 0, false, true, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Axis rightAxis{RIGHT_BUS_ID, -1, 0, 0, false, true, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void stateResetAll() {
  armed = false, confirmed = false, latched = false, zeroed = false, enabled = false;
  homeL = 0, homeR = 0, baseL = 0, baseR = 0;
  testSpeedLevel = 0;
  intentSinceL = 0, intentSinceR = 0;
}

void latch(const char *reason) {
  armed = false;
  latched = true;
  Serial.printf("LATCHED: %s; use independent physical cutoff if unsafe. Reset required.\n", reason);
  rsStopBothMotors();
}

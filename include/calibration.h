#pragma once
#include "types.h"

// Stationary tare + encoder home capture (3 s, motors OFF). Sets zeroed on success.
void tare();
// Copy homeL/R and baseL/R into the Axis structs.
void syncHomes();

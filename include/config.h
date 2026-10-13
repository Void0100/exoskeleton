// config.h - all compile-time constants. Edit here directly.Stationary
// All values are provisional FIXED-BENCH numbers, NOT validated safety limits.
// HX counts are NOT N/kg.
#pragma once
#include <stdint.h>
#include "commands.h"

// ---- Pins and bus addresses ----
static constexpr int TX_PIN = 21;  // UART1 TX to RS-485 transceiver, send signal to motors
static constexpr int RX_PIN = 20;  // UART1 RX from RS-485 transceiver, receive signal from moters
static constexpr int DIR_PIN = 8;  // RS-485 driver enable (HIGH=transmit, LOW=receive)
static constexpr int LEFT_FORCE_DATA_PIN = 5;  // Left HX711 data
static constexpr int LEFT_FORCE_CLOCK_PIN = 4;  // Left HX711 clock
static constexpr int RIGHT_FORCE_DATA_PIN = 2;  // Right HX711 data
static constexpr int RIGHT_FORCE_CLOCK_PIN = 3;  // Right HX711 clock
static constexpr uint8_t LEFT_BUS_ID = 1;  // Left motor bus address
static constexpr uint8_t RIGHT_BUS_ID = 2;  // Right motor bus address

// ---- Module activation ----
static constexpr bool USE_FORCE_SENSORS = false;

// ---- Timing and timeouts ----
static constexpr uint32_t RS_TIMEOUT_US = 20000; // Max time (us) to receive one RS-485 reply frame
static constexpr uint32_t GAP_US = 2500;  // Pause (us) after each bus transaction
static constexpr uint32_t HX_TIMEOUT_MS = 145;  // forces(): max wait for both HX711 ready (blocking path only)
static constexpr uint32_t ZERO_MS = 3000;  // Stationary tare window
static constexpr uint32_t HX_MAX_STALE_MS = 500;  // Sample older than this on one side latches (sustained HX loss)
static constexpr uint32_t INTENT_CONFIRM_MS = 30;  // Force must persist this long before a jog starts
static constexpr uint32_t SPEED_RAMP_MS = 120;  // Interval to re-evaluate speed/target while running
static constexpr uint32_t IDLE_DIAG_MS = 1000;  // Idle FORCE_IDLE_DIAG print interval
static constexpr uint32_t MAX_RUN_MS = 125000;  // Max duration of one jog before latching

// ---- Force thresholds (raw HX711 counts, NOT N or kg) ----
static constexpr double ENTER_COUNTS = 10000.0;  // Baseline-subtracted force to declare intent
static constexpr double EXIT_COUNTS = 5000.0;  // Force below this (in intent direction) ends a jog

// ---- Mechanics and travel limits ----
static constexpr double GEAR = 36.0;  // Motor-to-output ratio
static constexpr double FORWARD_LIMIT_DEG = 110.0;  // Software forward limit (output deg from home), provisional
static constexpr double BACKWARD_LIMIT_DEG = 15.0;  // Software backward limit (output deg from home), provisional
static constexpr double ANGLE_MARGIN = 0.5;  // Extra tolerance (output deg) in inside()

// ---- Speed selection ----
static constexpr double TEST_SPEED_LEVEL_DPS[4] = {30.0, 60.0, 90.0, 120.0};  // Selectable joint speeds (deg/s) = 5/10/15/20 RPM output; key 1..N
static constexpr uint8_t TEST_SPEED_LEVEL_DPS_COUNT = sizeof(TEST_SPEED_LEVEL_DPS) / sizeof(TEST_SPEED_LEVEL_DPS[0]);
static constexpr double TARGET_LOOKAHEAD_S = 0.3;  // Moving target leads current position by this many seconds of travel
static constexpr double MIN_LOOKAHEAD_OUTPUT_DEG = 8.0;  // Lower clamp on lead (output deg)
static constexpr double MAX_LOOKAHEAD_OUTPUT_DEG = 20.0;  // Upper clamp on lead (output deg)

// ---- Fault limits ----
static constexpr int MAX_SPEED_FB = 5000;  // Motor speed feedback ceiling (motor deg/s)
static constexpr int MAX_IQ_FB = 40;  // Torque-current limit; 3 consecutive exceedances latch
static constexpr int MAX_TEMP = 60;  // Motor temperature limit (C)
static constexpr double STOP_REVERSE_RAW_DEG = 3.0;  // Motion opposite to command beyond this latches (raw motor deg)
static constexpr int STOP_SPEED_FB = 3;  // Speed feedback at or below this counts as stopped

// ---- Legacy / currently unused (kept so nothing is lost) ----
static constexpr uint32_t JOG_TIMEOUT_MS = 125000;  // Superseded by MAX_RUN_MS
static constexpr uint32_t HX_PAIR_WAIT_WARN_MS = 40;  // Intended pair-wait warning
static constexpr uint8_t HX_MAX_CONSECUTIVE_FAILURES = 3;  // Counter was never incremented
static constexpr uint32_t SPEED_UPDATE_MS = 120;  // Old version leftover
static constexpr uint32_t STOP_SAMPLE_MS = 120;  // Old version leftover
static constexpr uint32_t STOP_MAX_MS = 900;  // Old version leftover
static constexpr uint32_t SPEED_DIAGNOSTIC_MS = 480;  // Old version leftover
static constexpr double STOP_DRIFT_RAW_DEG = 1.0;  // Planned run-on limit, not enforced
static constexpr double STOP_NET_RAW_DEG = 18.0;  // Planned run-on limit, not enforced

// // ---- Compile-time sanity checks (edit constants above, the build tells you if they conflict) ----
// constexpr double maxOfArray(const double *a, unsigned n) {
//   double m = a[0];
//   for (unsigned i = 1; i < n; i++) if (a[i] > m) m = a[i];
//   return m;
// }
// static_assert(ENTER_COUNTS > EXIT_COUNTS, "ENTER_COUNTS must exceed EXIT_COUNTS (hysteresis)");
// static_assert(MIN_LOOKAHEAD_OUTPUT_DEG <= MAX_LOOKAHEAD_OUTPUT_DEG, "lookahead min must be <= max");
// static_assert(GEAR > 0, "GEAR must be positive");
// static_assert(FORWARD_LIMIT_DEG > 0 && BACKWARD_LIMIT_DEG >= 0, "travel limits: FORWARD>0, BACKWARD>=0");
// static_assert(TEST_SPEED_LEVEL_DPS_COUNT >= 1 && TEST_SPEED_LEVEL_DPS_COUNT <= 9, "1..9 speed levels (keys 1..9)");
// static_assert(maxOfArray(TEST_SPEED_LEVEL_DPS, TEST_SPEED_LEVEL_DPS_COUNT) * GEAR <= MAX_SPEED_FB,
//               "fastest speed level x GEAR exceeds MAX_SPEED_FB; runtime speed fault would trip");

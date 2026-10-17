/* V50.6.5 modular split: STOP-CHAR FINALIZE FIX + AXIS STATE TELEMETRY + INDEPENDENT HX.
   FIXED UNLOADED BENCH ONLY. NOT wearable, NOT a safety controller.
   Independent physical power cutoff and hard stops are mandatory.
   HX counts are NOT N/kg. All thresholds are experimental.

   Force sensors: USE_FORCE_SENSORS (config.h).
     true  : HX711 modules drive the axes (normal operation).
     false : HX711 is never touched. Force is the ONLY thing that starts a jog, so with
             sensors off the motors never move; tare (home only), read, status, telemetry,
             arm and enable still work for testing the bus and the motors' telemetry.

   Commands: type 'help'. '!' = instant emergency stop (no Enter needed). */
#include <Arduino.h>
#include <strings.h> // strcasecmp
#include "config.h"
#include "types.h"
#include "state.h"
#include "rs485.h"
#include "hx711.h"
#include "calibration.h"
#include "axis.h"
#include "stopchar.h"
#include "commands.h"


// ---- State local to main -------------------------------------------------

// ---- Log timers ----
static uint32_t lastTelemetryMs = -TELEMETRY_INTERVAL_MS;
static uint32_t lastIdleDiagMs = -IDLE_DIAG_INTERVAL_MS;

static double forceLeft = 0, forceRight = 0; // latest force in counts, positive = forward (0 if sensors off)


static bool controlActive() { return stateArmed && stateZeroed && stateConfirmed && stateEnabled; }

// // ===========================================================================
// // Banner / help
// // ===========================================================================
// void printBanner() {
//   log_i("V50.6.5 FIXED BENCH: force sensors %s; speed level=%u => %.1f deg/s (%.1f RPM) joint; levels 1..%u (disarmed only). Type 'help'.",
//     USE_FORCE_SENSORS ? "ON" : "OFF (axes cannot start)",
//     (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0,
//     (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
//   commandsPrintHelp();
// }


// ===========================================================================
// Periodic services
// ===========================================================================
static void serviceAxisStateTelemetry() {
  uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS)
    return;
  lastTelemetryMs = now;

  MotorStatus leftMotorStatus{}, rightMotorStatus{};
  double leftMotorAngle = NAN, rightMotorAngle = NAN;
  bool lStatus = rsReadMotorStatus(LEFT_BUS_ID, leftMotorStatus);
  bool rStatus = rsReadMotorStatus(RIGHT_BUS_ID, rightMotorStatus);
  bool lAngle = rsReadAngle(LEFT_BUS_ID, leftMotorAngle);
  bool rAngle = rsReadAngle(RIGHT_BUS_ID, rightMotorAngle);

  Serial.printf(
    "AXIS_STATE "
    "L[state=%s moving=%d released=%d intent=%d speedCmd=%lu speedFB=%d raw=%.2f out=%.2f]\n"
    "R[state=%s moving=%d released=%d intent=%d speedCmd=%lu speedFB=%d raw=%.2f out=%.2f]\n",
    axisStateName(leftAxis), leftAxis.moving, leftAxis.released, leftAxis.intent,
    (unsigned long)leftAxis.speed, lStatus ? leftMotorStatus.speed : 9999,
    lAngle ? leftMotorAngle : NAN, lAngle ? outputDeg(leftAxis, leftMotorAngle) : NAN,
    axisStateName(rightAxis), rightAxis.moving, rightAxis.released, rightAxis.intent,
    (unsigned long)rightAxis.speed, rStatus ? rightMotorStatus.speed : 9999,
    rAngle ? rightMotorAngle : NAN, rAngle ? outputDeg(rightAxis, rightMotorAngle) : NAN);
}

// Real sensors. Returns true if at least one fresh sample was handled this pass.
static bool serviceForceSensors() {
  hxPhase = axesBusy() ? "AXIS_RUN" : "AXIS_IDLE";
  serviceIndependentHX(); // independent DATA-READY service, no forced L/R pair

  uint32_t now = millis();
  if (leftForceLatestMs && now - leftForceLatestMs > HX_MAX_STALE_MS) {
    Serial.printf("HX_STALE side=LEFT ageMs=%lu\n", (unsigned long)(now - leftForceLatestMs));
    latch("left force sensor timeout");
    return false;
  }
  if (rightForceLatestMs && now - rightForceLatestMs > HX_MAX_STALE_MS) {
    Serial.printf("HX_STALE side=RIGHT ageMs=%lu\n", (unsigned long)(now - rightForceLatestMs));
    latch("right force sensor timeout");
    return false;
  }
  if (!leftFresh && !rightFresh)
    return false;

  forceLeft = leftForceLatestMs ? -(leftLatestForceRaw - baseForceRawL) : 0.0; // left HX forward is negative
  forceRight = rightForceLatestMs ? (rightLatestForceRaw - baseForceRawR) : 0.0;

  if (controlActive()) {
    if (leftFresh) {
      leftFresh = false;
      updateAxis(leftAxis, forceLeft);
    }
    if (stateLatched)
      return true;
    if (rightFresh) {
      rightFresh = false;
      updateAxis(rightAxis, forceRight);
    }
  } else {
    leftFresh = false;
    rightFresh = false;
  }
  return true;
}

static void serviceIdleDiagnostics() {
  if (axesBusy() || (uint32_t)(millis() - lastIdleDiagMs) < IDLE_DIAG_MS)
    return;
  MotorStatus sl{}, sr{};
  double al = 0, ar = 0;

  if (DO_IDLE_DIAG) {
    if (!rsReadMotorStatus(LEFT_BUS_ID, sl) || !rsReadMotorStatus(RIGHT_BUS_ID, sr) ||
      !rsReadAngle(LEFT_BUS_ID, al) || !rsReadAngle(RIGHT_BUS_ID, ar)) {
      latch("idle telemetry failure");
    } else {
      if (USE_FORCE_SENSORS)
        log_d("FORCE_IDLE_DIAG L_force=%+.0f R_force=%+.0f Lraw=%.2f Rraw=%.2f Lstate=%s Rstate=%s\n",
          forceLeft, forceRight, al, ar, axisStateName(leftAxis), axisStateName(rightAxis));
      else
        log_d("IDLE_DIAG (force sensors off) Lraw=%.2f Rraw=%.2f Lstate=%s Rstate=%s\n",
          al, ar, axisStateName(leftAxis), axisStateName(rightAxis));
    }
  }
  lastIdleDiagMs = millis();
}

// ===========================================================================
// Arduino entry points
// ===========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  activateRxMode();
  if (USE_FORCE_SENSORS) {
    pinMode(LEFT_FORCE_DATA_PIN, INPUT);
    pinMode(RIGHT_FORCE_DATA_PIN, INPUT);
    pinMode(LEFT_FORCE_CLOCK_PIN, OUTPUT);
    pinMode(RIGHT_FORCE_CLOCK_PIN, OUTPUT);
    digitalWrite(LEFT_FORCE_CLOCK_PIN, LOW);
    digitalWrite(RIGHT_FORCE_CLOCK_PIN, LOW);
  }
  motorBus.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(250);

  reset();
}

void loop() {
  commandsPoll();
  serviceStopChars(); // must run, or a stopped axis never leaves STOP_MEASURE
  if (DO_PRINT_TELEMETRY)
    serviceAxisStateTelemetry();
  if (stateLatched)
    return;

  if (USE_FORCE_SENSORS) {
    bool gotSample = serviceForceSensors();
    if (stateLatched)
      return;
    if (!gotSample) {
      delayMicroseconds(100);
      return;
    }
  }
  // With sensors off nothing can start a jog; only the time-gated diagnostics run.
  serviceIdleDiagnostics();
}

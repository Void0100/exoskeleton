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

// ---- Local settings (move to config.h if you prefer) ---------------------
static constexpr uint32_t AXIS_STATE_TELEMETRY_INTERVAL_MS = 5000;

// ---- State local to main -------------------------------------------------
static uint32_t lastIdleDiagMs = 0;
static uint32_t axisStateLastPrintMs = 0;

static bool doPrintTelemetry = false;
static bool doIdleDiag = false;

static double forceLeft = 0, forceRight = 0; // latest force in counts, positive = forward (0 if sensors off)

static bool axesBusy() { return leftAxis.moving || rightAxis.moving; }
static bool controlActive() { return armed && zeroed && confirmed && enabled; }

// ===========================================================================
// Banner / help
// ===========================================================================
void helpV502()
{
  log_i("V50.6.5 FIXED BENCH: force sensors %s; speed level=%u => %.1f deg/s (%.1f RPM) joint; levels 1..%u (disarmed only). Type 'help'.",
        USE_FORCE_SENSORS ? "ON" : "OFF (axes cannot start)",
        (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0,
        (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
  commandsPrintHelp();
}

// ===========================================================================
// Command handlers
// ===========================================================================
static void emergencyStop() { latch("operator stop"); }

static void cmdHelp(const CommandArgs &) { commandsPrintHelp(); }

static void cmdStatus(const CommandArgs &)
{
  Serial.printf("STATUS armed=%d enabled=%d latched=%d tared=%d forceSensors=%s speed=level %u (%.1f deg/s) L=%s R=%s\n",
                armed, enabled, latched, zeroed, USE_FORCE_SENSORS ? "on" : "off",
                (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(),
                axisStateName(leftAxis), axisStateName(rightAxis));
}

static void cmdSpeed(const CommandArgs &args)
{
  if (args.count == 0)
  { // "speed" alone shows the current level
    Serial.printf("speed: level %u = %.1f joint-deg/s (%.1f RPM); levels 1..%u\n",
                  (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0,
                  (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
    return;
  }
  long level = 0;
  if (!parseLong(args.value[0], level, 1, TEST_SPEED_LEVEL_DPS_COUNT))
  {
    Serial.printf("ERR speed: '%s' is not a level between 1 and %u\n", args.value[0],
                  (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
    return;
  }
  if (armed || axesBusy())
  {
    Serial.println("ERR speed: denied while armed or moving; latch/reset or disarm first");
    return;
  }
  testSpeedLevel = (uint8_t)(level - 1);
  Serial.printf("OK speed: level %ld = %.1f joint-deg/s = %.1f RPM; motor command %.1f deg/s. Next: arm, enable\n",
                level, selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0, selectedMotorSpeed01() / 100.0);
}

static void cmdTare(const CommandArgs &)
{
  if (armed || axesBusy())
  {
    Serial.println("ERR tare: denied while armed or moving");
    return;
  }
  tare();
  if (zeroed)
    syncHomes();
}

static void cmdArm(const CommandArgs &)
{
  if (latched)
  {
    Serial.println("ERR arm: latched; reset the board");
    return;
  }
  if (!zeroed)
  {
    Serial.println("ERR arm: not tared; run 'tare' first");
    return;
  }
  if (armed)
  {
    Serial.println("ERR arm: already armed");
    return;
  }
  if (!rsStopBothMotors())
  {
    Serial.println("ERR arm: motor STOP/OFF not acknowledged");
    return;
  }
  armed = true;
  confirmed = true;
  enabled = false;
  leftAxis.released = rightAxis.released = false;
  intentSinceL = intentSinceR = 0;
  Serial.println("OK arm: armed; run 'enable' to allow independent axes");
}

static void cmdEnable(const CommandArgs &)
{
  if (latched)
  {
    Serial.println("ERR enable: latched; reset the board");
    return;
  }
  if (!armed)
  {
    Serial.println("ERR enable: not armed; run 'arm' first");
    return;
  }
  enabled = true;
  leftAxis.released = rightAxis.released = false;
  Serial.println(USE_FORCE_SENSORS
                     ? "OK enable: release both sensors once, then either side may trigger"
                     : "OK enable: force sensors are off, so no axis can start");
}

static void cmdRead(const CommandArgs &)
{
  if (axesBusy())
  {
    Serial.println("ERR read: denied while moving; use 'stop'");
    return;
  }
  int32_t l = 0, r = 0;
  double al = 0, ar = 0;
  if (USE_FORCE_SENSORS)
  {
    hxPhase = "MANUAL_PEEK";
    if (!readForceBoth(l, r))
    {
      Serial.println("ERR read: force sensor read failed");
      return;
    }
  }
  if (!rsReadAngle(LEFT_BUS_ID, al) || !rsReadAngle(RIGHT_BUS_ID, ar))
  {
    Serial.println("ERR read: motor angle read failed");
    return;
  }
  if (USE_FORCE_SENSORS)
    Serial.printf("FORCE L=%+.0f R=%+.0f counts ANGLE L=%.2f R=%.2f rawDeg\n", -(l - baseL), r - baseR, al, ar);
  else
    Serial.printf("FORCE sensors off. ANGLE L=%.2f R=%.2f rawDeg\n", al, ar);
}

static void cmdTelemetry(const CommandArgs &args)
{
  bool want = !doPrintTelemetry; // no argument = toggle
  if (args.count == 1)
  {
    if (strcasecmp(args.value[0], "on") == 0)
      want = true;
    else if (strcasecmp(args.value[0], "off") == 0)
      want = false;
    else
    {
      Serial.printf("ERR telemetry: '%s' is not 'on' or 'off'\n", args.value[0]);
      return;
    }
  }
  doPrintTelemetry = want;
  if (want)
    axisStateLastPrintMs = millis() - AXIS_STATE_TELEMETRY_INTERVAL_MS; // print right away
  Serial.printf("OK telemetry: AXIS_STATE printing %s (every %lu ms)\n", want ? "on" : "off",
                (unsigned long)AXIS_STATE_TELEMETRY_INTERVAL_MS);
}

static void cmdIdleDiag(const CommandArgs &args)
{
  bool want = !doIdleDiag; // no argument = toggle
  if (args.count == 1)
  {
    if (strcasecmp(args.value[0], "on") == 0)
      want = true;
    else if (strcasecmp(args.value[0], "off") == 0)
      want = false;
    else
    {
      Serial.printf("ERR idle diagnostic: '%s' is not 'on' or 'off'\n", args.value[0]);
      return;
    }
  }
  doIdleDiag = want;
  if (want)
    lastIdleDiagMs = millis() - IDLE_DIAG_MS; // print right away
  Serial.printf("OK idle diagnostic: forces printing %s (every %lu ms)\n", want ? "on" : "off",
                (unsigned long)AXIS_STATE_TELEMETRY_INTERVAL_MS);
}

static void cmdStop(const CommandArgs &) { emergencyStop(); }

// ===========================================================================
// Periodic services
// ===========================================================================
static void serviceAxisStateTelemetry()
{
  uint32_t now = millis();
  if (now - axisStateLastPrintMs < AXIS_STATE_TELEMETRY_INTERVAL_MS)
    return;
  axisStateLastPrintMs = now;

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
static bool serviceForceSensors()
{
  hxPhase = axesBusy() ? "AXIS_RUN" : "AXIS_IDLE";
  serviceIndependentHX(); // independent DATA-READY service, no forced L/R pair

  uint32_t now = millis();
  if (leftForceLatestMs && now - leftForceLatestMs > HX_MAX_STALE_MS)
  {
    Serial.printf("HX_STALE side=LEFT ageMs=%lu\n", (unsigned long)(now - leftForceLatestMs));
    latch("left force sensor timeout");
    return false;
  }
  if (rightForceLatestMs && now - rightForceLatestMs > HX_MAX_STALE_MS)
  {
    Serial.printf("HX_STALE side=RIGHT ageMs=%lu\n", (unsigned long)(now - rightForceLatestMs));
    latch("right force sensor timeout");
    return false;
  }
  if (!leftFresh && !rightFresh)
    return false;

  forceLeft = leftForceLatestMs ? -(leftLatestForceRaw - baseL) : 0.0; // left HX forward is negative
  forceRight = rightForceLatestMs ? (rightLatestForceRaw - baseR) : 0.0;

  if (controlActive())
  {
    if (leftFresh)
    {
      leftFresh = false;
      updateAxis(leftAxis, forceLeft);
    }
    if (latched)
      return true;
    if (rightFresh)
    {
      rightFresh = false;
      updateAxis(rightAxis, forceRight);
    }
  }
  else
  {
    leftFresh = false;
    rightFresh = false;
  }
  return true;
}

static void serviceIdleDiagnostics()
{
  if (axesBusy() || (uint32_t)(millis() - lastIdleDiagMs) < IDLE_DIAG_MS)
    return;
  MotorStatus sl{}, sr{};
  double al = 0, ar = 0;

  if (doIdleDiag) {
    if (!rsReadMotorStatus(LEFT_BUS_ID, sl) || !rsReadMotorStatus(RIGHT_BUS_ID, sr) ||
        !rsReadAngle(LEFT_BUS_ID, al) || !rsReadAngle(RIGHT_BUS_ID, ar))
    {
      latch("idle telemetry failure");
    }
    else
    {
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

static void reset() {

  stateResetAll();

  if (!rsStopBothMotors())
    latch("boot STOP/OFF failed");
  helpV502();
  if (!latched)
  {
    log_i("AUTO-TARE: fixed bench neutral, %s, motors OFF",
          USE_FORCE_SENSORS ? "both sensors unloaded/stationary" : "force sensors disabled (home only)");
    tare();
  }
  if (zeroed)
    syncHomes();
  Serial.println(zeroed && !latched
                     ? "READY DISARMED: choose 'speed', confirm cutoff/hard stops/clear rig; then 'arm' and 'enable'"
                     : "TARE FAILED: motors disarmed; run 'tare' to retry");
}

static void cmdReset(const CommandArgs &) {
  reset();
}

// terminal command lines
static const CommandSpec COMMANDS[] = {
    // name        usage                      help                                              min max handler
    {"help", "", "list commands", 0, 0, cmdHelp},
    {"status", "", "show arm/enable/latch state and axis states", 0, 0, cmdStatus},
    {"speed", "[level]", "show or set speed level (disarmed only)", 0, 1, cmdSpeed},
    {"tare", "", "zero force sensors and capture home (disarmed)", 0, 0, cmdTare},
    {"arm", "", "arm (sends STOP/OFF first)", 0, 0, cmdArm},
    {"enable", "", "let force start jogs (after arm)", 0, 0, cmdEnable},
    {"read", "", "print one force and angle sample", 0, 0, cmdRead},
    {"telemetry", "[on|off]", "toggle or set AXIS_STATE printing", 0, 1, cmdTelemetry},
    {"stop", "", "STOP/OFF and latch; reset required", 0, 0, cmdStop},
    {"reset", "", "Reset all state variables and calibrate", 0, 0, cmdReset},
    {"idleDiag", "[on|off]", "toggle or set IDLE DIAG printing", 0, 1, cmdIdleDiag},
};

// ===========================================================================
// Arduino entry points
// ===========================================================================
void setup()
{
  Serial.begin(115200);
  commandsInit(COMMANDS, sizeof(COMMANDS) / sizeof(COMMANDS[0]), emergencyStop);
  delay(1000);

  pinMode(DIR_PIN, OUTPUT);
  activateRxMode();
  if (USE_FORCE_SENSORS)
  {
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

void loop()
{
  commandsPoll();
  serviceStopChars(); // must run, or a stopped axis never leaves STOP_MEASURE
  if (doPrintTelemetry)
    serviceAxisStateTelemetry();
  if (latched)
    return;

  if (USE_FORCE_SENSORS)
  {
    bool gotSample = serviceForceSensors();
    if (latched)
      return;
    if (!gotSample)
    {
      delayMicroseconds(100);
      return;
    }
  }
  // With sensors off nothing can start a jog; only the time-gated diagnostics run.
  serviceIdleDiagnostics();
}

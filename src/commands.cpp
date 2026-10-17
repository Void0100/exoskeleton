#include "commands.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>  // strcasecmp

#include "state.h"
#include "axis.h"
#include "calibration.h"
#include "rs485.h"
#include "hx711.h"
#include "config.h"

static const CommandSpec* COMMANDS = nullptr;
static size_t commandCount = 0;
// static void (*emergencyStopCallback)() = latch("Forced termination");

static char lineBuffer[COMMAND_LINE_MAX];
static size_t lineLength = 0;
static bool lineOverflowed = false;







// ---- Command helpers ----

void reset() {

  resetAllStates();

  if (!rsStopBothMotors()) latch("boot STOP/OFF failed");
  
  // printBanner();
  if (stateLatched) { Serial.print("Reset FAILED : Latched. Something is wrong with the code."); return; }

  log_i("AUTO-TARE: fixed bench neutral, %s, motors OFF",
    USE_FORCE_SENSORS ? "both sensors unloaded/stationary" : "force sensors disabled (home only)");
  caliAndSyncHome();

  if (!stateCalibrated) { Serial.print("Reset FAILED : calibration unsuccessful. Something is wrong with the code."); return; }
  
  // Serial.println(stateZeroed && !stateLatched
  //   ? "READY DISARMED: choose 'speed', confirm cutoff/hard stops/clear rig; then 'arm' and 'enable'"
  //   : "TARE FAILED: motors disarmed; run 'tare' to retry");

  Serial.println("Reset DONE.");

  
}

static bool parseLong(const char* text, long& out, long minValue, long maxValue) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  errno = 0;
  long v = strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') return false;
  if (v < minValue || v > maxValue) return false;
  out = v;
  return true;
}

static bool parseDouble(const char* text, double& out, double minValue, double maxValue) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  errno = 0;
  double v = strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !isfinite(v)) return false;
  if (v < minValue || v > maxValue) return false;
  out = v;
  return true;
}

static const CommandSpec* findCommand(const char* name) {
  for (size_t i = 0; i < commandCount; i++) {
    if (strcasecmp(COMMANDS[i].name, name) == 0) return &COMMANDS[i];
  }
  return nullptr;
}

// Split the line in place into whitespace-separated tokens. Returns the token count,
// or -1 if there are too many.
static int tokenize(char* line, char* tokens[], size_t maxTokens) {
  size_t n = 0;
  char* p = line;
  while (*p) {
    while (*p && isspace((unsigned char)*p)) p++;
    if (!*p) break;
    if (n >= maxTokens) return -1;
    tokens[n++] = p;
    while (*p && !isspace((unsigned char)*p)) p++;
    if (*p) *p++ = '\0';
  }
  return (int)n;
}

static void executeLine(char* line) {
  char* tokens[1 + COMMAND_MAX_ARGS];
  int n = tokenize(line, tokens, 1 + COMMAND_MAX_ARGS);
  if (n < 0) {
    Serial.printf("ERR too many arguments (max %u after the command name)\n", (unsigned)COMMAND_MAX_ARGS);
    return;
  }
  if (n == 0) return;

  const CommandSpec* spec = findCommand(tokens[0]);
  if (spec == nullptr) {
    Serial.printf("ERR unknown command '%s'. Type 'help' for the list.\n", tokens[0]);
    return;
  }

  uint8_t argc = (uint8_t)(n - 1);
  if (argc < spec->minArgs || argc > spec->maxArgs) {
    if (spec->minArgs == spec->maxArgs) {
      Serial.printf("ERR %s: got %u argument(s), needs %u. Usage: %s %s\n",
        spec->name, argc, spec->minArgs, spec->name, spec->usage);
    } else {
      Serial.printf("ERR %s: got %u argument(s), needs %u to %u. Usage: %s %s\n",
        spec->name, argc, spec->minArgs, spec->maxArgs, spec->name, spec->usage);
    }
    return;
  }

  CommandArgs args;
  args.count = argc;
  for (uint8_t i = 0; i < COMMAND_MAX_ARGS; i++) args.value[i] = (i < argc) ? tokens[i + 1] : "";
  spec->handler(args);
}








// ===========================================================================
// Command handlers
// ===========================================================================
static void cmdHelp(const CommandArgs&) { 
  Serial.printf("Commands (case-insensitive; '%c' = emergency stop, no Enter needed):\n", EMERGENCY_STOP_CHAR);
  for (size_t i = 0; i < commandCount; i++) {
    const CommandSpec& c = COMMANDS[i];
    Serial.printf("  %-8s %-9s %s\n", c.name, c.usage, c.help);
  }
}

static void cmdStatus(const CommandArgs&) {
  Serial.printf("STATUS armed=%d enabled=%d latched=%d tared=%d forceSensors=%s speed=level %u (%.1f deg/s) L=%s R=%s\n",
    stateArmed, stateEnabled, stateLatched, stateCalibrated, USE_FORCE_SENSORS ? "on" : "off",
    (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(),
    axisStateName(leftAxis), axisStateName(rightAxis));
}

static void cmdSetSpeed(const CommandArgs& args) {
  if (args.count == 0) { // "speed" alone shows the current level
    Serial.printf("speed: level %u = %.1f joint-deg/s (%.1f RPM); levels 1..%u\n",
      (unsigned)(testSpeedLevel + 1), selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0,
      (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
    return;
  }
  long level = 0;
  if (!parseLong(args.value[0], level, 1, TEST_SPEED_LEVEL_DPS_COUNT)) {
    Serial.printf("ERR speed: '%s' is not a level between 1 and %u\n", args.value[0],
      (unsigned)TEST_SPEED_LEVEL_DPS_COUNT);
    return;
  }
  if (stateArmed || axesBusy()) {
    Serial.println("ERR speed: denied while armed or moving; latch/reset or disarm first");
    return;
  }
  testSpeedLevel = (uint8_t)(level - 1);
  Serial.printf("OK speed: level %ld = %.1f joint-deg/s = %.1f RPM; motor command %.1f deg/s. Next: arm, enable\n",
    level, selectedJointSpeedDps(), selectedJointSpeedDps() / 6.0, selectedMotorSpeed01() / 100.0);
}

static void cmdTare(const CommandArgs&) {
  if (stateArmed || axesBusy()) {
    Serial.println("ERR tare: denied while armed or moving");
    return;
  }
  caliAndSyncHome();
}

static void cmdArm(const CommandArgs&) {
  if (stateLatched) {
    Serial.println("ERR arm: latched; run reset");
    return;
  }
  if (!stateCalibrated) {
    Serial.println("ERR arm: not tared; run 'tare' first");
    return;
  }
  if (stateArmed) {
    Serial.println("ERR arm: already armed");
    return;
  }
  if (!rsStopBothMotors()) {
    Serial.println("ERR arm: motor STOP/OFF not acknowledged");
    return;
  }
  stateArmed = true;
  stateEnabled = false;
  leftAxis.released = rightAxis.released = false;
  intentSinceL = intentSinceR = 0;
  Serial.println("OK arm: armed; run 'enable' to allow independent axes");
}

static void cmdEnable(const CommandArgs&) {
  if (stateLatched) {
    Serial.println("ERR enable: latched; reset the board");
    return;
  }
  if (!stateArmed) {
    Serial.println("ERR enable: not armed; run 'arm' first");
    return;
  }
  stateEnabled = true;
  leftAxis.released = rightAxis.released = false;
  Serial.println(USE_FORCE_SENSORS
    ? "OK enable: release both sensors once, then either side may trigger"
    : "OK enable: force sensors are off, so no axis can start");
}

static void cmdRead(const CommandArgs&) {
  if (axesBusy()) {
    Serial.println("ERR read: denied while moving; use 'stop'");
    return;
  }
  int32_t l = 0, r = 0;
  double al = 0, ar = 0;
  if (USE_FORCE_SENSORS) {
    hxPhase = "MANUAL_PEEK";
    if (!readForceBoth(l, r)) {
      Serial.println("ERR read: force sensor read failed");
      return;
    }
  }
  if (!rsReadAngle(LEFT_BUS_ID, al) || !rsReadAngle(RIGHT_BUS_ID, ar)) {
    Serial.println("ERR read: motor angle read failed");
    return;
  }
  if (USE_FORCE_SENSORS)
    Serial.printf("FORCE L=%+.0f R=%+.0f counts ANGLE L=%.2f R=%.2f rawDeg\n", -(l - baseForceRawL), r - baseForceRawR, al, ar);
  else
    Serial.printf("FORCE sensors off. ANGLE L=%.2f R=%.2f rawDeg\n", al, ar);
}

static void cmdTelemetry(const CommandArgs& args) {
  bool want = !DO_PRINT_TELEMETRY; // no argument = toggle
  if (args.count == 1) {
    if (strcasecmp(args.value[0], "on") == 0)
      want = true;
    else if (strcasecmp(args.value[0], "off") == 0)
      want = false;
    else {
      Serial.printf("ERR telemetry: '%s' is not 'on' or 'off'\n", args.value[0]);
      return;
    }
  }
  DO_PRINT_TELEMETRY = want;
  Serial.printf("OK telemetry: AXIS_STATE printing %s (every %lu ms)\n", want ? "on" : "off",
    (unsigned long)TELEMETRY_INTERVAL_MS);
}

static void cmdIdleDiag(const CommandArgs& args) {
  bool want = !DO_IDLE_DIAG; // no argument = toggle
  if (args.count == 1) {
    if (strcasecmp(args.value[0], "on") == 0)
      want = true;
    else if (strcasecmp(args.value[0], "off") == 0)
      want = false;
    else {
      Serial.printf("ERR idle diagnostic: '%s' is not 'on' or 'off'\n", args.value[0]);
      return;
    }
  }
  DO_IDLE_DIAG = want;
  Serial.printf("OK idle diagnostic: forces printing %s (every %lu ms)\n", want ? "on" : "off",
    (unsigned long)IDLE_DIAG_INTERVAL_MS);
}

static void cmdStop(const CommandArgs&) { latch("Forced termination."); }

static void cmdReset(const CommandArgs&) {
  reset();
}



// ---- Command table -----------------------------------------------------
static const CommandSpec COMMAND_TABLE[] = {
  // name        usage                      help                                              min max handler
  { "help", "", "list commands", 0, 0, cmdHelp },
  { "status", "", "show arm/enable/latch state and axis states", 0, 0, cmdStatus },
  { "speed", "[level]", "show or set speed level (disarmed only)", 0, 1, cmdSetSpeed },
  { "tare", "", "zero force sensors and capture home (disarmed)", 0, 0, cmdTare },
  { "arm", "", "arm (sends STOP/OFF first)", 0, 0, cmdArm },
  { "enable", "", "let force start jogs (after arm)", 0, 0, cmdEnable },
  { "read", "", "print one force and angle sample", 0, 0, cmdRead },
  { "telemetry", "[on|off]", "toggle or set AXIS_STATE printing", 0, 1, cmdTelemetry },
  { "stop", "", "STOP/OFF and latch; reset required", 0, 0, cmdStop },
  { "reset", "", "Reset all state variables and calibrate", 0, 0, cmdReset },
  { "idleDiag", "[on|off]", "toggle or set IDLE DIAG printing", 0, 1, cmdIdleDiag },
};



// ---- Public functions --------------------------------------------------------------------------------------
void commandsPoll() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == EMERGENCY_STOP_CHAR) {   // checked before anything else, even mid-line
      lineLength = 0;
      lineOverflowed = false;
      latch("Forced termination");
      while (Serial.available()) (void)Serial.read();  // drop whatever else was typed
      return;
    }

    if (c == '\r' || c == '\n') {     // CR, LF or CRLF all end a line; empty lines are ignored
      if (lineOverflowed) {
        Serial.printf("ERR line too long (max %u characters), ignored\n", (unsigned)(COMMAND_LINE_MAX - 1));
      } else if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        executeLine(lineBuffer);
      }
      lineLength = 0;
      lineOverflowed = false;
      continue;
    }

    if (lineOverflowed) continue;
    if (lineLength + 1 < COMMAND_LINE_MAX) lineBuffer[lineLength++] = c;
    else lineOverflowed = true;
  }
}

#include "commands.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>  // strcasecmp

static const CommandSpec *commandTable = nullptr;
static size_t commandCount = 0;
static void (*emergencyStopCallback)() = nullptr;

static char lineBuffer[COMMAND_LINE_MAX];
static size_t lineLength = 0;
static bool lineOverflowed = false;

void commandsInit(const CommandSpec *table, size_t count, void (*emergencyStop)()) {
  commandTable = table;
  commandCount = count;
  emergencyStopCallback = emergencyStop;
  lineLength = 0;
  lineOverflowed = false;
}

void commandsPrintHelp() {
  Serial.printf("Commands (case-insensitive; '%c' = emergency stop, no Enter needed):\n", EMERGENCY_STOP_CHAR);
  for (size_t i = 0; i < commandCount; i++) {
    const CommandSpec &c = commandTable[i];
    Serial.printf("  %-8s %-9s %s\n", c.name, c.usage, c.help);
  }
}

bool parseLong(const char *text, long &out, long minValue, long maxValue) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  errno = 0;
  long v = strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') return false;
  if (v < minValue || v > maxValue) return false;
  out = v;
  return true;
}

bool parseDouble(const char *text, double &out, double minValue, double maxValue) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  errno = 0;
  double v = strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !isfinite(v)) return false;
  if (v < minValue || v > maxValue) return false;
  out = v;
  return true;
}

static const CommandSpec *findCommand(const char *name) {
  for (size_t i = 0; i < commandCount; i++) {
    if (strcasecmp(commandTable[i].name, name) == 0) return &commandTable[i];
  }
  return nullptr;
}

// Split the line in place into whitespace-separated tokens. Returns the token count,
// or -1 if there are too many.
static int tokenize(char *line, char *tokens[], size_t maxTokens) {
  size_t n = 0;
  char *p = line;
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

static void executeLine(char *line) {
  char *tokens[1 + COMMAND_MAX_ARGS];
  int n = tokenize(line, tokens, 1 + COMMAND_MAX_ARGS);
  if (n < 0) {
    Serial.printf("ERR too many arguments (max %u after the command name)\n", (unsigned)COMMAND_MAX_ARGS);
    return;
  }
  if (n == 0) return;

  const CommandSpec *spec = findCommand(tokens[0]);
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

void commandsPoll() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == EMERGENCY_STOP_CHAR) {   // checked before anything else, even mid-line
      lineLength = 0;
      lineOverflowed = false;
      if (emergencyStopCallback) emergencyStopCallback();
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

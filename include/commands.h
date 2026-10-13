#pragma once
#include <Arduino.h>

// ---- Limits -------------------------------------------------------------
static constexpr size_t COMMAND_LINE_MAX = 64;  // longest accepted line, including the terminator
static constexpr size_t COMMAND_MAX_ARGS = 4;   // arguments after the command name

// Typed anywhere in the input (no Enter needed) -> emergencyStop callback runs immediately.
static constexpr char EMERGENCY_STOP_CHAR = '!';

// ---- Types --------------------------------------------------------------
struct CommandArgs {
  uint8_t count;                          // number of arguments (name not included)
  const char *value[COMMAND_MAX_ARGS];    // argument text, valid only during the handler call
};

struct CommandSpec {
  const char *name;     // typed by the user, matched case-insensitively
  const char *usage;    // arguments only, e.g. "<level>" or "[level]"; "" if none
  const char *help;     // one-line description shown by 'help'
  uint8_t minArgs;
  uint8_t maxArgs;      // must be <= COMMAND_MAX_ARGS
  void (*handler)(const CommandArgs &args);
};

// ---- API ----------------------------------------------------------------
/** Register the command table (must outlive the program: use a static array). */
void commandsInit(const CommandSpec *table, size_t count, void (*emergencyStop)());

/** Read Serial and run complete lines. Call once per loop(); never blocks. */
void commandsPoll();

/** Print the command list built from the table. */
void commandsPrintHelp();

/**
 * Parse a whole-string base-10 integer within [minValue, maxValue].
 * @return false if the text is empty, has trailing characters, or is out of range
 *         (out is left unchanged).
 */
bool parseLong(const char *text, long &out, long minValue, long maxValue);

/** Same for a finite floating-point number. */
bool parseDouble(const char *text, double &out, double minValue, double maxValue);

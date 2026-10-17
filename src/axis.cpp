#include "axis.h"
#include "state.h"
#include "rs485.h"
#include "stopchar.h"

double outputDeg(const Axis &a, double raw) { return a.anatomicalToRaw * (raw - a.home) / GEAR; }

bool inside(const Axis &a, double raw) {
  double d = outputDeg(a, raw);
  return d >= -BACKWARD_LIMIT_DEG - ANGLE_MARGIN && d <= FORWARD_LIMIT_DEG + ANGLE_MARGIN;
}

// Intent encoding: 0 neutral, +1 forward, -1 backward. Counts are uncalibrated.
int classify(double forwardCounts) {
  if (forwardCounts >= ENTER_COUNTS) return +1;
  if (forwardCounts <= -ENTER_COUNTS) return -1;
  return 0;
}

uint32_t selectedMotorSpeed01() {
  double motorDps = TEST_SPEED_LEVEL_DPS[testSpeedLevel] * GEAR;
  return (uint32_t)llround(motorDps * 100.0);  // A4 speed unit = 0.01 motor-deg/s
}

double selectedJointSpeedDps() { return TEST_SPEED_LEVEL_DPS[testSpeedLevel]; }

double targetLeadRawDeg() {
  double outLead = selectedJointSpeedDps() * TARGET_LOOKAHEAD_S;
  outLead = max(MIN_LOOKAHEAD_OUTPUT_DEG, min(outLead, MAX_LOOKAHEAD_OUTPUT_DEG));
  return outLead * GEAR;
}

double targetFor(const Axis &a, int intent, double current) {
  double low = a.home + a.anatomicalToRaw * (-BACKWARD_LIMIT_DEG) * GEAR;
  double high = a.home + a.anatomicalToRaw * FORWARD_LIMIT_DEG * GEAR;
  double direction = a.anatomicalToRaw * intent;
  double candidate = current + direction * targetLeadRawDeg();
  return direction > 0 ? min(candidate, max(low, high)) : max(candidate, min(low, high));
}

bool stopAxis(Axis &a, const char *reason) {
  MotorStatus pre{};
  double posBefore = NAN;
  bool havePre = rsReadMotorStatus(a.id, pre) && rsReadAngle(a.id, posBefore);
  uint8_t b[32];
  size_t n = 0;
  bool st = rsSendCommand(a.id, Command::MotorStop, b, n), off = rsSendCommand(a.id, Command::MotorOff, b, n);
  a.moving = false; a.released = false; a.candidateCount = 0; a.intent = 0;
  Serial.printf("AXIS_STOP id=%u state=STOP_MEASURE reason=%s STOP=%d OFF=%d\n", a.id, reason, st, off);
  if (havePre) beginStopChar(a, posBefore, pre.speed);
  else Serial.printf("STOP_TEST_BEGIN id=%u PRESTOP_TELEMETRY_FAILED\n", a.id);
  if (!st || !off) { latch("axis stop ACK failure"); return false; }
  return true;
}

void startAxis(Axis &a, int intent, double force) {
  if (a.moving || stopCharFor(a.id).active || !a.released || !stateArmed || stateLatched || !stateEnabled || intent == 0) return;
  MotorStatus st{};
  double pos = 0;
  if (!rsReadMotorStatus(a.id, st) || !rsReadAngle(a.id, pos)) { latch("axis preflight telemetry failure"); return; }
  if (abs((int)st.speed) > 5 || !inside(a, pos)) { latch("axis preflight moving/outside limits"); return; }
  uint8_t b[32];
  size_t n = 0;
  if (!rsSendCommand(a.id, Command::MotorRun, b, n) || !rsSetTorqueZero(a.id)) {
    latch("axis enable failure");
    return;
  }
  a.start = a.lastAngle = pos;
  a.intent = intent;
  a.speed = selectedMotorSpeed01();
  a.target = targetFor(a, intent, pos);
  if (fabs(a.target - pos) < GEAR * 0.5) { stopAxis(a, "boundary at start"); return; }
  if (!a4(a.id, (int64_t)llround(a.target * 100), a.speed)) { latch("axis start A4 failure"); return; }
  a.moving = true;
  a.began = a.updated = millis();
  a.overIq = 0;
  Serial.printf("AXIS_START id=%u intent=%+d force=%+.0f start=%.2f target=%.2f\n", a.id, intent, force, pos, a.target);
}

void runAxis(Axis &a, double force) {
  if (!a.moving) return;
  int intent = classify(force);
  if (intent != a.intent || force * a.intent < EXIT_COUNTS) { stopAxis(a, "released/reversed"); return; }
  MotorStatus st{};
  double pos = 0;
  if (!rsReadMotorStatus(a.id, st) || !rsReadAngle(a.id, pos)) { latch("axis runtime telemetry failure"); return; }
  if (st.temperature >= MAX_TEMP || abs((int)st.speed) > MAX_SPEED_FB || !inside(a, pos)) {
    latch("axis temperature/speed/travel fault");
    return;
  }
  a.overIq = abs((int)st.iq) > MAX_IQ_FB ? a.overIq + 1 : 0;
  if (a.overIq >= 3) { latch("axis sustained IQ fault"); return; }
  double rawDir = a.anatomicalToRaw * a.intent;
  if (rawDir * (pos - a.lastAngle) < -STOP_REVERSE_RAW_DEG) { latch("axis unexpected reverse motion"); return; }
  if ((uint32_t)(millis() - a.began) > MAX_RUN_MS) { latch("axis run timeout"); return; }
  if ((uint32_t)(millis() - a.updated) >= SPEED_RAMP_MS) {
    uint32_t wanted = selectedMotorSpeed01();
    uint32_t next = wanted;
    double nextTarget = targetFor(a, a.intent, pos);
    if (rawDir * (nextTarget - pos) < GEAR * 0.5) { stopAxis(a, "soft boundary"); return; }
    if (next != a.speed || fabs(nextTarget - a.target) >= GEAR * 0.25) {
      if (!a4(a.id, (int64_t)llround(nextTarget * 100), next)) { latch("axis update A4 failure"); return; }
      a.speed = next;
      a.target = nextTarget;
    }
    a.updated = millis();
  }
  a.lastAngle = pos;
  Serial.printf("AXIS_RUN id=%u intent=%+d force=%+.0f output=%.3f jointCmd=%.1fRPM/%.1fdps motorCmd=%.1fdps speedFB=%d IQ=%d\n",
    a.id, a.intent, force, outputDeg(a, pos), selectedJointSpeedDps() / 6.0, selectedJointSpeedDps(), a.speed / 100.0, st.speed, st.iq);
}

void updateAxis(Axis &a, double force) {
  if (a.moving) { runAxis(a, force); return; }
  int intent = classify(force);
  uint32_t now = millis();
  uint32_t &since = (a.id == LEFT_BUS_ID) ? intentSinceL : intentSinceR;
  if (intent == 0) { a.released = true; a.candidateCount = 0; a.intent = 0; since = 0; return; }
  if (!a.released) return;
  if (intent != a.intent) { a.intent = intent; a.candidateCount = 1; since = now; return; }
  if (since == 0) since = now;
  if ((uint32_t)(now - since) >= INTENT_CONFIRM_MS) {
    a.candidateCount = 3;
    startAxis(a, intent, force);
    since = 0;
  }
}

bool axesBusy() { return leftAxis.moving || rightAxis.moving; }

const char *axisStateName(const Axis &a) {
  if (a.moving) return "RUN";
  if (stopCharFor(a.id).active) return "STOP_MEASURE";
  if (a.released) return "READY";
  return "WAIT_RELEASE";
}

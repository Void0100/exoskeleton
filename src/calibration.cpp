#include "calibration.h"
#include "state.h"
#include "rs485.h"
#include "hx711.h"
#include "commands.h"

static void printTareError(const char* msg) {
  Serial.printf("TARE DENIED: %s", msg);
}

bool caliAndSyncHome() {
  hxPhase = "TARE";
  if (stateLatched) { printTareError("latched"); return false; }
  if (stateArmed) { printTareError("armed"); return false; }
  stateCalibrated = false;
  double sl = 0, sr = 0;
  int32_t loL = INT32_MAX, hiL = INT32_MIN, loR = INT32_MAX, hiR = INT32_MIN;
  double minA[2] = { 1e99, 1e99 }, maxA[2] = { -1e99, -1e99 };
  unsigned n = 0;
  uint32_t t = millis();
  Serial.println("TARE/HOME: set both joints to FIXED BENCH NEUTRAL and keep sensors stationary for 3 seconds; motors OFF");
  while ((uint32_t)(millis() - t) < ZERO_MS) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == EMERGENCY_STOP_CHAR) {
        latch("operator stop during tare");
        return false;
      }
    }

    int32_t l = 0, r = 0;
    double al = 0, ar = 0;
    MotorStatus ml{}, mr{};
    if (USE_FORCE_SENSORS && !readForceBoth(l, r)) {
      Serial.println("TARE FAIL: read force failure");
      return false;
    }
    if (!rsReadMotorStatus(LEFT_BUS_ID, ml) || !rsReadMotorStatus(RIGHT_BUS_ID, mr) || !rsReadAngle(LEFT_BUS_ID, al) || !rsReadAngle(RIGHT_BUS_ID, ar)) {
      Serial.println("TARE FAIL: read angle failure");
      return false;
    }
    if (abs((int)ml.speed) > 15 || abs((int)mr.speed) > 15) { Serial.println("TARE FAIL: moving"); return false; }
    sl += l; sr += r; n++;
    if (l < loL) loL = l;
    if (l > hiL) hiL = l;
    if (r < loR) loR = r;
    if (r > hiR) hiR = r;
    if (al < minA[0]) minA[0] = al;
    if (al > maxA[0]) maxA[0] = al;
    if (ar < minA[1]) minA[1] = ar;
    if (ar > maxA[1]) maxA[1] = ar;
    delay(12);
  }
  if (n < 15 || maxA[0] - minA[0] > 3 || maxA[1] - minA[1] > 3) {
    Serial.println("TARE FAIL: insufficient samples or encoder drift");
    return false;
  }
  baseForceRawL = sl / n;
  baseForceRawR = sr / n;
  baseAngRawL = (minA[0] + maxA[0]) * 0.5;
  baseAngRawR = (minA[1] + maxA[1]) * 0.5;
  stateCalibrated = true;
  Serial.printf("TARE/HOME OK \nn=%u baseline raw force L=%.0f R=%.0f noise spread L=%ld R=%ld counts; NOT calibrated N/kg\n",
    n, baseForceRawL, baseForceRawR, (long)(hiL - loL), (long)(hiR - loR));
  Serial.printf("angle baseline L=%.0f R=%.0f noise spread L=%ld R=%ld counts; NOT calibrated deg\n",
    baseAngRawL, baseAngRawR, (long)(maxA[0] - minA[0]), (long)(maxA[1] - minA[1]));

  syncHomes();
  return true;
}

void syncHomes() {
  leftAxis.home = baseAngRawL;
  rightAxis.home = baseAngRawR;
  leftAxis.base = baseForceRawL;
  rightAxis.base = baseForceRawR;
}

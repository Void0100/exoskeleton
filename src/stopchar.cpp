#include "stopchar.h"
#include "state.h"
#include "rs485.h"
#include "axis.h"

StopChar stopL, stopR;

StopChar &stopCharFor(int id) { return id == LEFT_BUS_ID ? stopL : stopR; }

void beginStopChar(const Axis &a, double posBefore, int16_t speedBefore) {
  StopChar &q = stopCharFor(a.id);
  q = StopChar{};
  q.active = true;
  q.id = a.id;
  q.anatomicalToRaw = a.anatomicalToRaw;
  q.t0 = millis();
  q.pos0 = posBefore;
  q.speed0 = speedBefore;
  Serial.printf("STOP_TEST_BEGIN id=%u speedBeforeRaw=%d posAtStopRaw=%.2f outputAtStop=%.3f\n",
    q.id, q.speed0, q.pos0, outputDeg(a, q.pos0));
}

void serviceStopCharOne(StopChar &q, const Axis &a) {
  (void)a;
  if (!q.active) return;
  uint32_t e = millis() - q.t0;

  // Finalization must NOT be hidden behind the snapshot/settle "need" gate (V50.6.5 fix).
  if (e >= 550 && q.got500) {
    MotorStatus st{};
    double pos = NAN;
    if (!rsReadMotorStatus(q.id, st) || !rsReadAngle(q.id, pos)) return;
    bool haveSettled = (q.settledAt != 0);
    double finalPos = haveSettled ? q.settledPos : pos;
    uint32_t settle = haveSettled ? q.settledAt : e;
    Serial.printf("STOP_TEST_SUMMARY id=%u speedBeforeRaw=%d speedBeforeJoint=%.2f posAtStopRaw=%.2f finalRaw=%.2f overshootOutput=%.3f settleTimeMs=%lu settled=%d\n",
      q.id, q.speed0, ((double)q.speed0) / GEAR, q.pos0, finalPos,
      q.anatomicalToRaw * (finalPos - q.pos0) / GEAR, (unsigned long)settle, haveSettled);
    q.active = false;
    return;
  }

  bool need = (!q.got50 && e >= 50) || (!q.got100 && e >= 100) || (!q.got200 && e >= 200) ||
              (!q.got500 && e >= 500) || (q.settledAt == 0 && e >= 20);
  if (!need) return;

  MotorStatus st{};
  double pos = NAN;
  if (!rsReadMotorStatus(q.id, st) || !rsReadAngle(q.id, pos)) return;

  auto snap = [&](const char *tag, double &dst, bool &flag) {
    dst = pos;
    flag = true;
    Serial.printf("STOP_TEST id=%u %s raw=%.2f outputDelta=%.3f speedFB=%d\n",
      q.id, tag, pos, q.anatomicalToRaw * (pos - q.pos0) / GEAR, st.speed);
  };
  if (!q.got50 && e >= 50) snap("+50ms", q.p50, q.got50);
  if (!q.got100 && e >= 100) snap("+100ms", q.p100, q.got100);
  if (!q.got200 && e >= 200) snap("+200ms", q.p200, q.got200);
  if (!q.got500 && e >= 500) snap("+500ms", q.p500, q.got500);

  if (q.settledAt == 0) {
    if (abs((int)st.speed) <= STOP_SPEED_FB) q.stableCount++;
    else q.stableCount = 0;
    if (q.stableCount >= 3) { q.settledAt = e; q.settledPos = pos; }
  }
}

void serviceStopChars() {
  serviceStopCharOne(stopL, leftAxis);
  serviceStopCharOne(stopR, rightAxis);
}

#include "rs485.h"

HardwareSerial motorBus(1);

// ---- Frame layout: START | cmd | id | payloadLen | headerSum | payload... | payloadSum ----
static constexpr uint8_t FRAME_START = 0x3E;  // ASCII '>'
static constexpr size_t HEADER_SIZE = 5;

// ---- Motor calibration constants ----
static constexpr double MAX_CALI_DEG_DIFF = 20;


static uint8_t sumUntil(const uint8_t *arr, size_t end) {
  uint8_t s = 0;
  for (size_t i = 0; i < end; i++) s = (uint8_t)(s + arr[i]);
  return s;
}

const char* MotorCommand::toName() {
  switch (code) {
    case Code::MotorOff:       return "MotorOff";
    case Code::MotorStop:      return "MotorStop";
    case Code::MotorRun:       return "MotorRun";
    case Code::ReadAngle:      return "ReadAngle";
    case Code::ReadStatus:     return "ReadStatus";
    case Code::SetTorque:      return "SetTorque";
    case Code::MoveToPosition: return "MoveToPosition";
    default:                   return "Unknown";
  }
}

void activateRxMode() { // receiving mode
  digitalWrite(DIR_PIN, LOW);
}

void activateTxMode() { // sending mode
  digitalWrite(DIR_PIN, HIGH);
}

static void rsSendPacket(const uint8_t *packet, size_t size) {
  activateRxMode();
  while (motorBus.available()) (void)motorBus.read();
  activateTxMode();
  motorBus.write(packet, size); 
  motorBus.flush();
  activateRxMode();
}

static bool readReply(MotorCommand cmd, uint8_t id, uint8_t *b, size_t &sizeCnt) {
  sizeCnt = 0;
  uint32_t startUs = micros();
  auto timeRemaining = [&]() { return (uint32_t)(micros() - startUs) < RS_TIMEOUT_US; };
  while (timeRemaining()) {
    if (motorBus.available() && motorBus.read() == FRAME_START) {
      b[sizeCnt++] = FRAME_START;
      break;
    }
    yield();
  }
  if (!timeRemaining()) {
    log_e("Reading frame failed: time elapsed (%uus) exceeds RS_TIMEOUT_US (%uus). motorId=%u, cmd=%s", micros() - startUs, RS_TIMEOUT_US, id, cmd.toName());
  }
  if (sizeCnt == 0) {
    log_e("Reading frame failed: FRAME_START not received. motorId=%u, cmd=%s", id, cmd.toName());
    return false;
  }

  while (sizeCnt < HEADER_SIZE && timeRemaining()) {
    if (motorBus.available()) b[sizeCnt++] = (uint8_t)motorBus.read();
    else yield();
  }
  if (!timeRemaining()) {
    log_e("Reading frame failed: time elapsed (%uus) exceeds RS_TIMEOUT_US (%uus). motorId=%u, cmd=%s", micros() - startUs, RS_TIMEOUT_US, id, cmd.toName());
  }
  if (sizeCnt != HEADER_SIZE || b[1] != cmd.toByte() || b[2] != id || sumUntil(b, 4) != b[4]) {
    log_e("Reading frame failed: Some bullshit happened. motorId=%u, cmd=%s", id, cmd.toName());
    return false;
  }

  size_t len = b[3], total = 5 + len + (len ? 1 : 0);
  if (total > RS485_MAX_FRAME_SIZE) {
    log_e("Reading frame failed: MAX_FRAME_SIZE exceeded. motorId=%u, cmd=%s", id, cmd.toName());
    return false;
  }
  while (sizeCnt < total && timeRemaining()) {
    if (motorBus.available()) b[sizeCnt++] = (uint8_t)motorBus.read();
    else yield();
  }
  if (!timeRemaining()) {
    log_e("Reading frame failed: time elapsed (%uus) exceeds RS_TIMEOUT_US (%uus). motorId=%u, cmd=%s", micros() - startUs, RS_TIMEOUT_US, id, cmd.toName());
  }

  // check total frame size and payload sum
  return sizeCnt == total && (len == 0 || sumUntil(b + 5, len) == b[5 + len]);
}

bool rsSendCommand(uint8_t motorId, MotorCommand cmd, uint8_t *b, size_t &size) {
  uint8_t p[5] = {0};
  p[0] = FRAME_START; p[1] = cmd.toByte(); p[2] = motorId; p[3] = 0;
  p[4] = sumUntil(p, 4);
  rsSendPacket(p, 5);
  
  bool success = readReply(cmd, motorId, b, size);
  delayMicroseconds(GAP_US);
  if (!success) {
    log_d("Command to motor failed to send. motorID=%u, cmd=%s", motorId, cmd.name());
  }
  return success;
}

bool rsReadMotorStatus(uint8_t motorId, MotorStatus &stat) {
  uint8_t b[RS485_MAX_FRAME_SIZE];
  size_t n = 0;
  if (!rsSendCommand(motorId, MotorCommand::Code::ReadStatus, b, n) || n != 13 || b[3] != 7)
    return false;
  stat.temperature = (int8_t)b[5];
  stat.iq = (int16_t)((uint16_t)b[6] | ((uint16_t)b[7] << 8));
  stat.speed = (int16_t)((uint16_t)b[8] | ((uint16_t)b[9] << 8));
  stat.encoder = (uint16_t)b[10] | ((uint16_t)b[11] << 8);
  return true;
}

bool rsReadAngle(uint8_t id, double &deg) {
  uint8_t b[RS485_MAX_FRAME_SIZE];
  size_t n = 0;
  if (!rsSendCommand(id, MotorCommand::Code::ReadAngle, b, n) || n != 14 || b[3] != 8)
    return false;
  uint64_t raw = 0;
  for (int i = 0; i < 8; i++) raw |= (uint64_t)b[5 + i] << (8 * i);
  deg = (double)(int64_t)raw * 0.01;
  return true;
}

bool rsSetTorqueZero(uint8_t motorId) {
  uint8_t b[RS485_MAX_FRAME_SIZE];
  size_t n = 0;
  if (!rsSendCommand(motorId, MotorCommand::Code::SetTorque, b, n) || n != 13 || b[3] != 7)
    return false;
  return true;
}

bool rsMoveToPos(uint8_t id, int64_t targetPos, uint32_t speedLimit) {
  MotorCommand cmd = MotorCommand::Code::MoveToPosition;
  uint8_t p[18] = {FRAME_START, cmd.toByte(), id, 12, 0};
  p[4] = sumUntil(p, 4);
  uint64_t bits = (uint64_t)targetPos;
  for (int i = 0; i < 8; i++) p[5 + i] = (uint8_t)(bits >> (8 * i));
  for (int i = 0; i < 4; i++)
    p[13 + i] = (uint8_t)(speedLimit >> (8 * i));
  p[17] = sumUntil(p + 5, 12);
  rsSendPacket(p, 18);
  uint8_t b[32];
  size_t n = 0;
  bool ok = readReply(cmd, id, b, n);
  delayMicroseconds(GAP_US);
  return ok && n == 13 && b[3] == 7;
}

bool rsStop(uint8_t motorId) {
  uint8_t b[RS485_MAX_FRAME_SIZE];
  size_t sizeCnt = 0;
  bool succ = rsSendCommand(motorId, MotorCommand::Code::MotorStop, b, sizeCnt);
  if (!succ) {
    Serial.printf("Motor stop failed, motorId=%u \n", motorId);
  }
  return true;
}

bool rsOff(uint8_t motorId) {
  uint8_t b[RS485_MAX_FRAME_SIZE];
  size_t sizeCnt = 0;
  bool succ = rsSendCommand(motorId, MotorCommand::Code::MotorOff, b, sizeCnt);
  if (!succ) {
    Serial.printf("Motor stop failed, motorId=%u \n", motorId);
  }
  return true;
}

// bool rsStopBothMotors() {
//   uint8_t b[RS485_MAX_FRAME_SIZE];
//   size_t n = 0;
//   bool leftMotorStop = rsSendCommand(LEFT_BUS_ID, Command::MotorStop, b, n);
//   bool leftMotoroff = rsSendCommand(LEFT_BUS_ID, Command::MotorOff, b, n);
//   bool rightMotorStop = rsSendCommand(RIGHT_BUS_ID, Command::MotorStop, b, n);
//   bool rightMotoroff = rsSendCommand(RIGHT_BUS_ID, Command::MotorOff, b, n);
//   log_e("STOP/OFF L=%s/%s R=%s/%s (ACK is NOT physical isolation)",
//         leftMotorStop ? "true" : "false",
//         leftMotoroff ? "true" : "false",
//         rightMotorStop ? "true" : "false",
//         rightMotoroff ? "true" : "false");
//   return leftMotorStop && leftMotoroff && rightMotorStop && rightMotoroff;
// }

Motor::Motor(uint8_t motorId) {
  motorId = motorId;
}

uint8_t Motor::getMotorId() {
  return motorId;
}
bool Motor::getMotorStatus(MotorStatus &stat) {
  return rsReadMotorStatus(motorId, stat);
}
bool Motor::getMotorAngle(double &deg) {
  return rsReadAngle(motorId, deg);
}
bool Motor::setTorqueZero() {
  return rsSetTorqueZero(motorId);
}
bool Motor::moveToPos(int64_t targetPos, uint32_t speedLimit) {
  return rsMoveToPos(motorId, targetPos, speedLimit);
}
bool Motor::latch() {
  if (stateLatched) { Serial.println("Already latched"); return true; }
  bool stopped = rsStop(motorId);
  bool isOff = rsOff(motorId);
  if (!(stopped && isOff)) {
    Serial.printf("Motor latch failed, motorId=%u \n", motorId);
  }
  stateLatched = true;
  return true;
}

bool Motor::calibrate() {
  if (stateLatched) { log_e("calibrate while latched, motorId=%u", motorId); return false; }
  if (stateArmed) { log_e("calibrate while armed, motorId=%u", motorId); return false; }
  stateCalibrated = false;
  uint32_t t = millis();
  Serial.println("TARE/HOME: set both joints to FIXED BENCH NEUTRAL and keep sensors stationary for 3 seconds; motors OFF");

  int cnt = 0;
  double totAngle = 0;
  double minA = 1e99; double maxA = -1e99;
  while ((uint32_t)(millis() - t) < ZERO_MS) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == EMERGENCY_STOP_CHAR) {
        latch();
        return false;
      }
    }

    double angle = 0;
    if (!rsReadAngle(motorId, angle)) {
      Serial.println("MOTOR CALIBRATION FAIL: read angle failure");
      return false;
    }
    if (angle < minA) { minA = angle; }
    if (angle > maxA) { maxA = angle; }

    if (maxA - minA > MAX_CALI_DEG_DIFF) { Serial.println("MOTOR CALIBRATE FAIL: moving or encoder drift"); return false; }
    cnt++;
    totAngle += angle;
    delay(12);
  }

  if (cnt < 15) {
    Serial.println("TARE FAIL: insufficient samples");
    return false;
  }

  baseAngle = totAngle / cnt;
  stateCalibrated = true;

  Serial.printf("MOTOR CALIBRATION SUCESSFUL, motorId=%u, \nno. of sample=%u baseAngle=%.0f spread R=%.0f.",
    motorId, cnt, baseAngle, maxA-minA);
  return true;
}

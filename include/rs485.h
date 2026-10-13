#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

extern HardwareSerial motorBus;

// Largest frame the receive buffers can hold. Use it to size every reply buffer.
static constexpr size_t RS485_MAX_FRAME_SIZE = 32;

// Motor commands (the byte after the 0x3E start marker).
enum class Command : uint8_t {
  MotorOff = 0x80,
  MotorStop = 0x81,
  MotorRun = 0x88,       // enable, sent before the first move
  ReadAngle = 0x92,      // multi-turn angle
  ReadStatus = 0x9C,     // temperature, IQ, speed, encoder
  SetTorque = 0xA1,      // torque/current command (used with 0 to zero it)
  MoveToPosition = 0xA4  // absolute position with a speed limit
};

const char *commandName(Command cmd);

constexpr uint8_t toByte(Command c) {
  return static_cast<uint8_t>(c);
}

void activateRxMode();  // bus receive mode
void activateTxMode();  // bus send mode

bool rsSendCommand(uint8_t id, Command cmd, uint8_t* b, size_t& n);
bool rsReadMotorStatus(uint8_t id, MotorStatus &s);        
bool rsReadAngle(uint8_t id, double &deg);              
bool rsSetTorqueZero(uint8_t id);                          
bool a4(uint8_t id, int64_t target01, uint32_t speed01); // absolute position + speed limit
bool rsStopBothMotors();                                  // STOP(0x81)+OFF(0x80) both; ACK is NOT isolation

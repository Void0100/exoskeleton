#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

extern HardwareSerial motorBus;

// Largest frame the receive buffers can hold. Use it to size every reply buffer.
static constexpr size_t RS485_MAX_FRAME_SIZE = 32;

// motor angle conversion
static constexpr double ANGLE_RAW_TO_DEG = 0.01;
static constexpr double GEAR_RATIO = 36;

// Motor commands (the byte after the 0x3E start marker).
struct MotorCommand {
  enum class Code : uint8_t {
    MotorOff = 0x80,
    MotorStop = 0x81,
    MotorRun = 0x88,       // enable, sent before the first move
    ReadAngle = 0x92,      // multi-turn angle
    ReadStatus = 0x9C,     // temperature, IQ, speed, encoder
    SetTorque = 0xA1,      // torque/current command (use 0 to zero it)
    MoveToPosition = 0xA4  // absolute position with a speed limit
  };

  Code code;

  MotorCommand(Code code) {
    code = code;
  }

  constexpr uint8_t toByte() {
    return static_cast<uint8_t>(code);
  }

  const char* toName();
};

void activateRxMode();  // bus receive mode
void activateTxMode();  // bus send mode

bool rsSendCommand(uint8_t id, MotorCommand cmd, uint8_t* b, size_t& n);
bool rsReadMotorStatus(uint8_t id, MotorStatus &s);        
bool rsReadAngleDeg(uint8_t id, double &deg);              
bool rsSetTorqueZero(uint8_t id);                          
bool rsMoveToPos(uint8_t id, int64_t target01, uint32_t speed01); // absolute position + speed limit
// bool rsStopBothMotors();                                  // STOP(0x81)+OFF(0x80) both; ACK is NOT isolation
// void sendPacket(const uint8_t *packet, size_t size);
bool rsOff(uint8_t motorId);
bool rsStop(uint8_t motorId);

class Motor {
  private:
    uint8_t motorId;
    bool stateArmed = false, stateLatched = false, stateCalibrated = false, stateEnabled = false;
    double baseAngleDeg = 0.;
    
  public:
    Motor(uint8_t motorId);

    static double rawAngleToDeg(uint64_t rawA);

    // get states, just returning the private states of the motor, do it this way to keep them read only
    bool isArmed();
    bool isLatched();
    bool isCalibrated();
    bool isEnabled();

    uint8_t getMotorId();

    // motor actions that require command sending and blocks motorbus. The docs below may not match what the function does, need testing TODO
    /** return MotorStatus {
     * int8_t temperature : in celcius
     * int16_t iq : torque, in current A
     * int16_t speed : in deg/s, as in the real axis version, including consideration of the gear ratio
     * uint16_t encoder : not sure what this is, TODO : check
     * }
     */
    bool getMotorStatus(MotorStatus &stat);
    /** return motor current position(deg), also should match real life/axis like above */
    bool getMotorAngleDeg(double &deg);
    /** ok this is just same as motorOff, change this to setTorque and accept torque TODO */
    bool setTorqueZero();
    /** move to position(deg) at speedLimit(deg/s), also should match real life */
    bool moveToPos(int64_t targetPos, uint32_t speedLimit);

    bool doLatch();
    bool doArm();
    bool doCalibrate();
    bool doEnable();
    bool doReset();
};
#include "axis.h"
#include "state.h"
#include "rs485.h"
#include "stopchar.h"
#include "hx711.h"

// double outputDeg(const Axis &a, double raw) { return a.anatomicalToRaw * (raw - a.home) / GEAR; }

// const char *axisStateName(const Axis &a) {
//   if (a.moving) return "RUN";
//   if (stopCharFor(a.id).active) return "STOP_MEASURE";
//   if (a.released) return "READY";
//   return "WAIT_RELEASE";
// }















// ---- Axis class functions ----
// get states
bool Axis::isArmed() { return motor.isArmed(); }
bool Axis::isLatched() { return motor.isLatched(); }
bool Axis::isCalibrated() { return motor.isCalibrated(); }
bool Axis::isEnabled() { return motor.isEnabled(); }

// sets and gets
bool Axis::getMotorStatus(MotorStatus &stat) { return motor.serviceGetStatus(stat); }




// state actions
bool Axis::doLatch() { return motor.doLatch(); }
bool Axis::doArm() { return motor.doArm(); }
bool Axis::doCalibrate() { return motor.doCalibrate(); }
bool Axis::doEnable() { return motor.doEnable(); }

// constraints, cant move over bound and stuff, not sure how to do that yet
/**checks if the motor's angle is within designed movement range */
// bool isInbound();


// modes (pls god pls work fuck)
bool Axis:: initBoundedSwing(double speedDeg) {
  _swingDt.dir = 1; _swingDt.speedDeg = speedDeg;
  return true;
}

bool Axis::stepBoundedSwing() {
  // TODO : fail cases handling
  double speed; double angle;
  motor.serviceGetSpeed(speed);
  motor.serviceGetAngDeg(angle);

  if (speed == 0) { // if stopped, switch dir
    _swingDt.dir *= -1;
    log_d("Axis %s changed direction to %s; Final pos=%f.0 deg",
          name, (_modeBoundedSwingData.dir==1) ? "forward" : "backward", angle);
    
    double target = (_swingDt.dir == 1) ? FORWARD_LIMIT_DEG : BACKWARD_LIMIT_DEG;
    motor.moveToPos(target, _swingDt.speedDeg*_swingDt.dir);
  }
  return true;
}


bool Axis::initAmpSpeed(double ampRatio, double speedLimDeg) {
  if (ampRatio < 1) return false;
  _ampSpeedDt.ampRatio = ampRatio;
  _ampSpeedDt.speedLimDeg = speedLimDeg;
  _ampSpeedDt.currMotorSpeed = 0;
  return true;
}

bool Axis::stepAmpSpeed() {
  double speed;
  motor.serviceGetSpeed(speed);
  if (speed < 2.) { // stopped
    if (_ampSpeedDt.currMotorSpeed > 2.) {
      if (!motor.setTorqueZero()) {
        return false;
      }
      return true;
    }
    return true;
  }
  if (abs(speed-_ampSpeedDt.currMotorSpeed) > 2.) { // if speed diff of user and motor gen > 2 deg/s
    // update motor speed
    double newMotorSpeed = speed*_ampSpeedDt.ampRatio;
    if (abs(newMotorSpeed) > _ampSpeedDt.speedLimDeg) {
      newMotorSpeed = _ampSpeedDt.speedLimDeg*(speed < 1 ? -1 : 1);
    }
    if (!motor.moveToPos(newMotorSpeed > 0 ? FORWARD_LIMIT_DEG : BACKWARD_LIMIT_DEG, newMotorSpeed)) {
      return false;
    }

    _ampSpeedDt.currMotorSpeed = newMotorSpeed;
    return true;
  }
}

void Axis::changeMode(AxisMode mode) {
  mode = mode;
  switch (mode)
  {
  case AxisMode::BoundedSwing:
    initBoundedSwing(30.);
  case AxisMode::AmpSpeed:
    initAmpSpeed(1.5, 60.);
  default:
    break;
  }
}


// update
/** update motor and force sensor readings */
bool Axis::update() {
  if (!motor.update()) {
    log_e("motor update failed");
    return false;
  }
  forceSensor.serviceUpdateForce();
  return true;
}

// execute next step according to current readings, unsure about this design tho
bool Axis::step() {
  switch (mode)
  {
  case AxisMode::BoundedSwing:
    stepBoundedSwing();
  case AxisMode::AmpSpeed:
    stepAmpSpeed();
  default:
    break;
  }
}

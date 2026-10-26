#pragma once
#include <Arduino.h>

#include "config.h"
#include "types.h"
#include "hx711.h"
#include "rs485.h"

// /*
//  * Axis control: turns a force reading into a bounded motor jog.
//  *
//  * Units used throughout this module:
//  *   raw    = motor-shaft degrees (what the encoder reports; 36 raw deg = 1 output deg)
//  *   output = joint degrees relative to the home captured by tare()
//  *   force  = baseline-subtracted HX711 counts, sign-corrected so that
//  *            positive = anatomical forward. NOT newtons or kg.
//  *   intent = -1 backward, 0 neutral, +1 forward
//  *
//  * Per-axis states (see axisStateName): WAIT_RELEASE -> READY -> RUN -> STOP_MEASURE -> WAIT_RELEASE
//  */

// /**
//  * Joint angle of a motor position, relative to home.
//  * @param a    axis (supplies home and direction sign)
//  * @param raw  motor angle in raw degrees
//  * @return     output degrees; positive = anatomical forward
//  */
// double outputDeg(const Axis& a, double raw);

// /**
//  * Whether a motor position is inside the software travel limits.
//  * Range is -BACKWARD_LIMIT_DEG to +FORWARD_LIMIT_DEG output degrees,
//  * widened by ANGLE_MARGIN on both ends. Provisional limits, not verified safe.
//  * @param raw  motor angle in raw degrees
//  */
// bool inside(const Axis& a, double raw);

// /**
//  * Convert a force reading into an intent.
//  * @param forwardCounts  force in counts, positive = forward
//  * @return +1 if >= ENTER_COUNTS, -1 if <= -ENTER_COUNTS, otherwise 0 (dead band).
//  * Note: the lower EXIT_COUNTS threshold that ends a jog is applied in runAxis(), not here.
//  */
// int classify(double forwardCounts);

// /**
//  * Selected speed as an A4 command value.
//  * @return motor deg/s x 100 (the A4 unit is 0.01 motor-deg/s).
//  *         Example: level of 30 joint-deg/s -> 30 x 36 x 100 = 108000.
//  */
// uint32_t selectedMotorSpeed01();

// /** Selected speed in joint (output) degrees per second, from TEST_SPEED_LEVEL_DPS. */
// double selectedJointSpeedDps();

// /**
//  * How far ahead of the current position a moving target is placed.
//  * Equals speed x TARGET_LOOKAHEAD_S, clamped to MIN/MAX_LOOKAHEAD_OUTPUT_DEG.
//  * @return distance in raw degrees (always positive; direction is applied by targetFor)
//  */
// double targetLeadRawDeg();

// /**
//  * Target position for a jog, stopped at the software travel limit.
//  * @param intent   +1 or -1
//  * @param current  current motor angle in raw degrees (assumed to be inside the limits)
//  * @return target in raw degrees: current plus the lookahead, clamped at the limit
//  */
// double targetFor(const Axis& a, int intent, double current);

// /**
//  * Stop one motor and begin recording how far it coasts (stop characterization).
//  * Reads status and angle first, then sends STOP and OFF, and clears moving, released and intent.
//  * The axis must then see a neutral force again before it can restart.
//  * STOP/OFF may let gravity backdrive the joint.
//  * @param reason  short text printed in the AXIS_STOP line
//  * @return true if both STOP and OFF were acknowledged.
//  *         On a missing ACK it calls latch() (both motors off, reset required) and returns false.
//  */
// bool stopAxis(Axis& a, const char* reason);

// /**
//  * Begin a jog. Does nothing (silently) unless all of these hold: not moving, no stop
//  * measurement running, released, armed, enabled, not latched, intent != 0.
//  * Sequence: read status and angle, enable the motor (0x88), zero torque, send the first A4.
//  * @param force  used only for the AXIS_START log line
//  * Latches on: telemetry failure, speed above 5 or position outside the limits at start,
//  * enable failure, or A4 failure. If under 0.5 output degree of travel is left, it stops
//  * instead of starting.
//  */
// void startAxis(Axis& a, int intent, double force);

// /**
//  * Supervise a running jog. Call once per fresh force sample. Does nothing if the axis is not moving.
//  * Stops normally (not a latch) when force is released below EXIT_COUNTS, the intent reverses,
//  * or less than 0.5 output degree remains before the soft limit.
//  * Latches on: telemetry failure, temperature >= MAX_TEMP, speed over MAX_SPEED_FB,
//  * position outside the limits, 3 consecutive IQ readings over MAX_IQ_FB,
//  * reverse motion beyond STOP_REVERSE_RAW_DEG, run time over MAX_RUN_MS, or A4 update failure.
//  * Every SPEED_RAMP_MS it recomputes the target and resends A4 if the speed changed
//  * or the target moved by at least 0.25 output degree.
//  * Costs two RS-485 transactions per call.
//  * @param force  current force in counts, positive = forward
//  */
// void runAxis(Axis& a, double force);

// /**
//  * Per-axis state machine. Call once per fresh force sample for this axis.
//  * - Moving: hands over to runAxis().
//  * - Neutral force: marks the axis released and resets the confirmation timer.
//  * - Not yet released: ignores the force (prevents starting on a push that was already there).
//  * - A new intent starts a timer; once it has held for INTENT_CONFIRM_MS, startAxis() is called.
//  * @param force  current force in counts, positive = forward
//  */
// void updateAxis(Axis& a, double force);

// bool axesBusy();

// /**
//  * Current state as text for logs.
//  * @return "RUN", "STOP_MEASURE", "READY" or "WAIT_RELEASE" (checked in that priority order)
//  */
// const char* axisStateName(const Axis& a);

// struct Axis {
//   uint8_t id;
//   int anatomicalToRaw;      // +1 left, -1 right
//   double home, base;        // encoder home (raw deg), HX baseline (counts)
//   bool moving, released;
//   int intent;               // -1, 0, +1
//   uint8_t candidateCount;
//   double target, start, lastAngle;
//   uint32_t speed, began, updated;
//   uint8_t overIq;
// };

enum class AxisMode : uint8_t { Idle, BoundedSwing, AmpSpeed };

class Axis {
private:
    Motor &motor;
    ForceSensor &forceSensor;
    const char* name;

    /** is the current motor angle within bounded movement range */
    bool isMovingOutbound(double posDeg, double lookAheadDeg);


    // MODES. (IMPORTANT, ENTIRE LOGIC PROGRAMMED ALL HERE BASICALLY)
    AxisMode mode;

    /** Every mode should be programmed in the following structure:
     *      1, a data struct : for storing current mode's variable data after every step taken
     *      2, the private _data variable
     *      3, the mode initializer, set the first condition before looping steps
     *      4, the step function itself, should be called periodically, maybe by Axis.step or sth
     */

     // mode BoundedSwing: move back and forth within the set bound, used to test if the bound is working
    struct SwingData {
        int dir;
        double speedDeg;
    };
    SwingData _swingDt;
    bool initBoundedSwing(double speedDeg);
    bool stepBoundedSwing();

    // mode ampSpeed: should move motor at speed faster than user speed
    struct AmpSpeedData {
        double ampRatio; // > 1
        double speedLimDeg;
        double currMotorSpeed;
    };
    AmpSpeedData _ampSpeedDt;
    bool initAmpSpeed(double ampRatio, double speedLimDeg);
    bool stepAmpSpeed();

public:
    Axis(const char* axisName, Motor &motor, ForceSensor &forceSensor) :
        name(axisName),
        motor(motor),
        forceSensor(forceSensor) {}

    // get states
    bool isArmed();
    bool isLatched();
    bool isCalibrated();
    bool isEnabled();

    // sets and gets
    bool getMotorStatus(MotorStatus& stat);
    bool getAngle(double& angle);

    // state actions
    bool doLatch();
    bool doArm();
    bool doCalibrate();
    bool doEnable();

    // constraints, cant move over bound and stuff, not sure how to do that yet
    /**checks if the motor's angle is within designed movement range */
    // bool isInsideAngleBound(double marginDeg);


    // modes (pls god pls work fuck)
    void changeMode(AxisMode mode);

    // update
    /** update motor and force sensor readings */
    bool update();

    // execute next step according to current readings, unsure about this design tho
    bool step();

};
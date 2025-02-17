#ifndef CONSTANTS_H
#define CONSTANTS_H

// #include "generated/TunerConstants.h"
#include "units/acceleration.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace DriveConstants {
// Initial Tuner constants
constexpr units::meters_per_second_t kMaxSpeed =
    5.22_mps;  // copied from "generated/TunerConstants.h"
// TunerConstants::kSpeedAt12Volts;  // kSpeedAt12Volts desired top speed
constexpr units::radians_per_second_t kMaxAngularRate = 1.2_tps;

// Controller dampening constants
const double kControllerVelocityCurveExponent = 2.0;
const double kControllerRotationCurveExponent = 2.0;
const double kControllerDeadbandPercentage = 0.02;

// Softens the behavior of sudden controller input
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
constexpr units::meters_per_second_squared_t kSlewTranslateLimit =
    2.0 * kMaxSpeed / 1_s;
constexpr units::radians_per_second_squared_t kSlewRotateLimit =
    3.0 * kMaxAngularRate / 1_s;

const double kRollerMovementForwardVelocity = 0.6;
const double kRollerMovementBackwardVelocity = -0.6;

const double kArmUpVelocity = 0.6;    /* 1.0 = maximum speed */
const double kArmDownVelocity = -0.2; /* 1.0 = maximum speed */

// Arm position constants
const double kArmIntakePosition = -1500;
const double kArmHoldPosition = -400;
const double kArmCoralEjectPosition = -300;
const double kArmDefaultPosition = 100;

// Arm motor output constants
const double kArmMotorForwardNominalPercentOutput = 0.0;
const double kArmMotorReverseNominalPercentOutput = 0.0;
const double kArmMotorForwardPeakPercentOutput = 0.3;
const double kArmMotorReversePeakPercentOutput = -0.3;

// Arm motor motion constants
const double kArmMotorMagicMotionCruiseVelocity = 50.0;
const double kArmMotorMagicMotionAccelerationVelocity = 50.0;

// Arm motor PID constants
const double kArmMotorProportionalGainValue = 2.0;  // Proportional gain
const double kArmMotorIntegralGainValue = 0.0;      // Integral gain
const double kArmMotorDerivativeGainValue = 5.0;    // Derivative gain
const double kArmMotorFeedForwardGainValue = 0.1;   // Feedforward gain
const double kArmSelectedSensorPosition = 0.0;
const double kArmMotorAllowableCloseLoopError = 5.0;

// Auto Commands
const auto kAutoIntakeAlgaeWait = 1_s;
const auto kAutoEjectAlgaeWait = 1_s;
const auto kAutoEjectCoralWait = 0.3_s;

// Intake sequence wait times
const auto kAlgaeIntakeSequenceWait = 0.2_s;
const auto kArmCoralEjectSequenceWait = 0.3_s;

/*
 * Hold down button #2 when plugging in to set the mode to Xbox mode (Default).
 * https://archive.org/details/img20240818_14140413/mode/2up
 *
 *          +--------+
 *          | POV=0  |
 *          | Coral  |          +------------+-----------+----------+----------+
 *          |  Out   |          | 3) Roller  | 4) Roller | 6) Arm   | 5) Arm   |
 * +--------+--------+-------+  |  backward  |  Forward  |    Up    |    Down  |
 * | POV270 | POV180 | POV90 |  +------------+-----------+----------+----------+
 * | Align  | Align  | Align |  | 1) Unclimb | 2) Climb  | A3 Algae | A2 Algae |
 * | Left   | Center | Right |  |            |           |   Eject  |   Intake |
 * +--------+--------+-------+  +------------+-----------+----------+----------+
 *                       +-------+
 *                       | POV=0 |
 *                       | Coral |
 *                       |  Out  |
 *                       +-------+
 */

// Button board
const int kClimbButton = 2;
const int kUnclimbButton = 1;
const int kRollerForwardButton = 4;
const int kRollerBackwardButton = 3;
const int kArmUpButton = 6;
const int kArmDownButton = 5;

const int kAlgaeIntakeButtonAxis = 2;
const int kAlgaeEjectButtonAxis = 3;
/* Coral out is POV=0 */
/* AutoAlign is POV=90-270 */

}  // namespace DriveConstants

#endif  // CONSTANTS_H
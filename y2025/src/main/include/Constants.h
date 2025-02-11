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
const double kControllerRotationCurveExponent = 4.0;
const double kControllerDeadbandPercentage = 0.02;

// Softens the behavior of sudden controller input
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
constexpr units::meters_per_second_squared_t kSlewTranslateLimit =
    2.0 * kMaxSpeed / 1_s;
constexpr units::radians_per_second_squared_t kSlewRotateLimit =
    3.0 * kMaxAngularRate / 1_s;

const units::second_t kArmMovementPeriod = 0.5_s;
const double kArmMovementForwardVelocity = 0.6;    /* 1.0 = maximum speed */
const double kRollerMovementForwardVelocity = 1;   /* 1.0 = maximum speed */
const double kRollerMovementBackwardVelocity = -1; /* 1.0 = maximum speed */

const units::second_t kArmMovementHoldPeriod = 0.3_s;
const double kArmMovementBackwardVelocity = -0.6; /* 1.0 = maximum speed */
const double kArmMovementEjectVelocity = 0.6;     /* 1.0 = maximum speed */

// Arm angle constants
// 90 degrees is vertical and 0 degrees is horizontal
const int kAlgaeIntakeArmDegree = 30;
const int kAlgaeHoldArmDegree = 70;
const int kAlgaeEjectArmDegree = 30;
const int kDefaultArmDegree = 90;

// Button board
const int kClimbButton = 4;
const int kUnclimbButton = 2;

const int kAlgaeIntakeButton = 3;
const int kAlgaeEjectButton = 5;
const int kCoralEjectButton = 6;

}  // namespace DriveConstants

#endif  // CONSTANTS_H
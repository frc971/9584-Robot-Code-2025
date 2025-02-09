#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "generated/TunerConstants.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace DriveConstants {
// Initial Tuner constants
constexpr units::meters_per_second_t kMaxSpeed =
    TunerConstants::kSpeedAt12Volts;  // kSpeedAt12Volts desired top speed
constexpr units::radians_per_second_t kMaxAngularRate = 1.2_tps;

// Controller dampening constants
const double kControllerVelocityCurveExponent = 2.0;
const double kControllerRotationCurveExponent = 4.0;
const double kControllerDeadbandPercentage = 0.02;

// Softens the behavior of sudden controller input
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
constexpr auto kSlewTranslateLimit = 2.0 * kMaxSpeed / 1_s;
constexpr auto kSlewRotateLimit = 3.0 * kMaxAngularRate / 1_s;

// Button board
const int kClimbButton = 4;
const int kUnclimbButton = 2;
}  // namespace DriveConstants

#endif  // CONSTANTS_H
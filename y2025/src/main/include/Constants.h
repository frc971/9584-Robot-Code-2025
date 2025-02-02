#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "units/base.h"
namespace DriveConstants {
    // Initial Tuner constants
    constexpr units::meters_per_second_t kMaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    constexpr units::radians_per_second_t kMaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    // Controller dampening constants
    const float kControllerVelocityCurveExponent = 2.0;
    const float kControllerRotationCurveExponent = 2.0;
    const double kControllerDeadbandPercentage = 0.02;

    // Softens the behavior of sudden controller input https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    constexpr auto kSlewTranslateLimit = 0.6 * kMaxSpeed / 1_s;
    constexpr auto kSlewRotateLimit = 1.0 * kMaxAngularRate / 1_s;

    // Button board
    const int kClimbButton = 4;
    const int kUnclimbButton = 2;
}


#endif // CONSTANTS_H
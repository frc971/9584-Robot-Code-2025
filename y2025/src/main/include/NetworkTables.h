#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTable.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Constants.h"

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class NetworkTables {
 public:
  /*************************************************************************
   * The order must match the items in constantEntries in NetworkTables.cpp.
   *************************************************************************/
  enum class ConstantId {
    MaxSpeed,                         // type: units::meters_per_second_t
    MaxAngularRate,                   // type: units::radians_per_second_t
    ControllerVelocityCurveExponent,  // type: double
    ControllerRotationCurveExponent,  // type: double
    ControllerDeadbandPercentage,     // type: double
    SlewTranslateLimit,             // type: units::meters_per_second_squared_t
    SlewRotateLimit,                // type: units::radians_per_second_squared_t
    RollerMovementHoldVelocity,     // type: double
    RollerMovementForwardVelocity,  // type: double
    RollerMovementBackwardVelocity,           // type: double
    RollerMovementCoralEjectVelocity,         // type: double
    RollerMovementAlgaeIntakeVelocity,        // type: double
    RollerMovementAlgaeEjectVelocity,         // type: double
    ArmUpVelocity,                            // type: double
    ArmDownVelocity,                          // type: double
    ArmIntakePosition,                        // type: double
    ArmHoldPosition,                          // type: double
    ArmCoralEjectPosition,                    // type: double
    ArmDefaultPosition,                       // type: double
    ArmMotorForwardNominalPercentOutput,      // type: double
    ArmMotorReverseNominalPercentOutput,      // type: double
    ArmMotorForwardPeakPercentOutput,         // type: double
    ArmMotorReversePeakPercentOutput,         // type: double
    ArmMotorMagicMotionCruiseVelocity,        // type: double
    ArmMotorMagicMotionAccelerationVelocity,  // type: double
    ArmMotorProportionalGainValue,            // type: double
    ArmMotorIntegralGainValue,                // type: double
    ArmMotorDerivativeGainValue,              // type: double
    ArmMotorFeedForwardGainValue,             // type: double
    ArmSelectedSensorPosition,                // type: double
    ArmMotorAllowableCloseLoopError,          // type: double
    ClimbVelocity,                            // type: double
    UnclimbVelocity,                          // type: double
    ClimberTorqueCurrentLimit,                // type: units::current::ampere_t
    AutoIntakeAlgaeWait,                      // type: units:seconds_t
    AutoEjectAlgaeWait,                       // type: units:seconds_t
    AutoEjectCoralWait,                       // type: units:seconds_t
    AlgaeIntakeSequenceWait,                  // type: units:seconds_t
    ArmCoralEjectSequenceWait,                // type: units:seconds_t
    ClimbButton,                              // type: int
    UnclimbButton,                            // type: int
    RollerForwardButton,                      // type: int
    RollerBackwardButton,                     // type: int
    ArmUpButton,                              // type: int
    ArmDownButton,                            // type: int
    ResetEncoderButton,                       // type: int
    AlgaeIntakeButtonAxis,                    // type: int
    AlgaeEjectButtonAxis,                     // type: int
    kNumConstants  // Always last; not used as a constant.
  };
  NetworkTables();
  void RestoreDefaults();

  double getDoubleValue(ConstantId id);
  int getIntValue(ConstantId id);
  bool getBooleanValue(ConstantId id);
  std::string getStringValue(ConstantId id);
  units::radians_per_second_t getAngularRateValue(ConstantId id);
  units::meters_per_second_squared_t getAccelerationValue(ConstantId id);
  units::radians_per_second_squared_t getAngularAccelerationValue(
      ConstantId id);
  units::time::second_t getTimeValue(ConstantId id);
  units::velocity::meters_per_second_t getVelocityValue(ConstantId id);
  units::current::ampere_t getCurrentValue(ConstantId id);

 private:
  std::shared_ptr<nt::NetworkTable> table;
  const std::string kTableName = "Tuning Constants";

  nt::BooleanSubscriber resetSub;
  NT_Listener resetListenerHandle;

  // Internal enum to tag the type of each constant.
  // Add the corresponding getXXXValue in NetworkTables.cpp
  enum class ConstantType {
    Double,               // double
    Boolean,              // boolean
    String,               // string
    Int,                  // stored as double; converted to int
    Velocity,             // stored as double; converted to m/s
    AngularRate,          // stored as double; converted to rad/s
    Acceleration,         // stored as double; converted to m/s²
    AngularAcceleration,  // stored as double; converted to rad/s²
    Time,                 // stored as double; converted to seconds
    Current,              // stored as double; converted to ampere
  };

  using CT = NetworkTables::ConstantType;

  union DefaultValue {
    double doubleValue;
    bool boolValue;
    const char* stringValue;
  };

  struct ConstantEntry {
    std::string networkTableKey;
    ConstantType type;
    DefaultValue defaultValue;
  };

  static const std::array<ConstantEntry,
                          static_cast<size_t>(ConstantId::kNumConstants)>
      constantEntries;
};
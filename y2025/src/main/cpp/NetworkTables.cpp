#include "NetworkTables.h"

#include <math.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/velocity.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>

#include "generated/TunerConstants.h"

const units::meters_per_second_t kMaxSpeed = TunerConstants::kSpeedAt12Volts;
const double kMaxAngularRate = 1.2;

/***********************************************************************
 * The order must match the items in enum ConstantId in NetworkTables.h.
 ***********************************************************************/
const std::array<NetworkTables::ConstantEntry,
                 static_cast<size_t>(NetworkTables::ConstantId::kNumConstants)>
    NetworkTables::constantEntries{{
        {"MaxSpeed", CT::Velocity, {.doubleValue = kMaxSpeed.value()}},
        {"MaxAngularRate",
         CT::AngularRate,
         {.doubleValue = kMaxAngularRate * 2 * M_PI}},
        // Controller dampening constants
        {"ControllerVelocityCurveExponent", CT::Double, {.doubleValue = 2.0}},
        {"ControllerRotationCurveExponent", CT::Double, {.doubleValue = 2.0}},
        {"ControllerDeadbandPercentage", CT::Double, {.doubleValue = 0.02}},
        // Softens the behavior of sudden controller input
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
        {"SlewTranslateLimit",
         CT::Acceleration,
         {.doubleValue = 10.0 * kMaxSpeed.value()}},
        {"SlewRotateLimit",
         CT::AngularAcceleration,
         {.doubleValue = 30.0 * kMaxAngularRate}},
        {"RollerMovementHoldVelocity", CT::Double, {.doubleValue = 0.05}},
        {"RollerMovementForwardVelocity", CT::Double, {.doubleValue = 0.6}},
        {"RollerMovementBackwardVelocity", CT::Double, {.doubleValue = -0.6}},
        {"RollerMovementCoralEjectVelocity", CT::Double, {.doubleValue = 1.0}},
        {"RollerMovementAlgaeIntakeVelocity", CT::Double, {.doubleValue = 0.5}},
        {"RollerMovementAlgaeEjectVelocity", CT::Double, {.doubleValue = -0.6}},
        /* 1.0 = maximum speed */
        {"ArmUpVelocity", CT::Double, {.doubleValue = 0.6}},
        /* 1.0 = maximum speed */
        {"ArmDownVelocity", CT::Double, {.doubleValue = -0.2}},
        // Arm position constants
        {"ArmIntakePosition", CT::Double, {.doubleValue = -1600}},
        {"ArmHoldPosition", CT::Double, {.doubleValue = -500}},
        {"ArmCoralEjectPosition", CT::Double, {.doubleValue = -700}},
        // this will apply constant voltage to the motor to keep the arm upright
        // when in the vertical position
        {"ArmDefaultPosition", CT::Double, {.doubleValue = 700}},
        // Arm motor output constants
        {"ArmMotorForwardNominalPercentOutput",
         CT::Double,
         {.doubleValue = 0.0}},
        {"ArmMotorReverseNominalPercentOutput",
         CT::Double,
         {.doubleValue = 0.0}},
        {"ArmMotorForwardPeakPercentOutput", CT::Double, {.doubleValue = 0.4}},
        {"ArmMotorReversePeakPercentOutput", CT::Double, {.doubleValue = -0.4}},
        // Arm motor motion constants
        {"ArmMotorMagicMotionCruiseVelocity",
         CT::Double,
         {.doubleValue = 50.0}},
        {"ArmMotorMagicMotionAccelerationVelocity",
         CT::Double,
         {.doubleValue = 50.0}},
        // Arm motor PID constants
        {"ArmMotorProportionalGainValue", CT::Double, {.doubleValue = 5.0}},
        {"ArmMotorIntegralGainValue", CT::Double, {.doubleValue = 0.0}},
        {"ArmMotorDerivativeGainValue", CT::Double, {.doubleValue = 5.0}},
        {"ArmMotorFeedForwardGainValue", CT::Double, {.doubleValue = 0.1}},
        {"ArmSelectedSensorPosition", CT::Double, {.doubleValue = 0.0}},
        {"ArmMotorAllowableCloseLoopError", CT::Double, {.doubleValue = 5.0}},
        // Climb motor constants
        {"ClimbVelocity", CT::Double, {.doubleValue = 1}},
        {"UnclimbVelocity", CT::Double, {.doubleValue = -1}},
        {"ClimberTorqueCurrentLimit", CT::Current, {.doubleValue = 22}},
        // Auto Commands
        {"AutoIntakeAlgaeWait", CT::Time, {.doubleValue = 1}},
        {"AutoEjectAlgaeWait", CT::Time, {.doubleValue = 1}},
        {"AutoEjectCoralWait", CT::Time, {.doubleValue = 0.5}},
        // Intake sequence wait times
        {"AlgaeIntakeSequenceWait", CT::Time, {.doubleValue = 0.01}},
        {"ArmCoralEjectSequenceWait", CT::Time, {.doubleValue = 0.2}},  // 0.1
        // Button board
        {"ClimbButton", CT::Int, {.doubleValue = 2}},
        {"UnclimbButton", CT::Int, {.doubleValue = 1}},
        {"RollerForwardButton", CT::Int, {.doubleValue = 4}},
        {"RollerBackwardButton", CT::Int, {.doubleValue = 3}},
        {"ArmUpButton", CT::Int, {.doubleValue = 6}},
        {"ArmDownButton", CT::Int, {.doubleValue = 5}},
        {"ResetEncoderButton", CT::Int, {.doubleValue = 8}},
        {"AlgaeIntakeButtonAxis", CT::Int, {.doubleValue = 2}},
        {"AlgaeEjectButtonAxis", CT::Int, {.doubleValue = 3}},
        /* Coral out is POV=0 */
        /* AutoAlign is POV=90-270 */
    }};

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

NetworkTables::NetworkTables() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  table = inst.GetTable(kTableName);

  for (const auto &entry : constantEntries) {
    switch (entry.type) {
      case CT::Double:               // double
      case CT::Int:                  // stored as double; converted to int
      case CT::Velocity:             // stored as double; converted to m/s
      case CT::AngularRate:          // stored as double; converted to rad/s
      case CT::Acceleration:         // stored as double; converted to m/s²
      case CT::AngularAcceleration:  // stored as double; converted to rad/s²
      case CT::Time:                 // stored as double; converted to seconds
      case CT::Current:              // stored as double; converted to ampere
        table->SetDefaultNumber(entry.networkTableKey,
                                entry.defaultValue.doubleValue);
        break;
      case CT::Boolean:  // boolean
        table->SetDefaultBoolean(entry.networkTableKey,
                                 entry.defaultValue.boolValue);
        break;
      case CT::String:  // string
        table->SetDefaultString(entry.networkTableKey,
                                entry.defaultValue.stringValue);
        break;
      default:
        throw std::runtime_error("Set Defaults: Unsupported constant type");
    }
  }
}

void NetworkTables::RestoreDefaults() {
  for (const auto &entry : constantEntries) {
    switch (entry.type) {
      case CT::Double:               // double
      case CT::Int:                  // stored as double; converted to int
      case CT::Velocity:             // stored as double; converted to m/s
      case CT::AngularRate:          // stored as double; converted to rad/s
      case CT::Acceleration:         // stored as double; converted to m/s²
      case CT::AngularAcceleration:  // stored as double; converted to rad/s²
      case CT::Time:                 // stored as double; converted to seconds
      case CT::Current:              // stored as double; converted to ampere
        table->PutNumber(entry.networkTableKey, entry.defaultValue.doubleValue);
        break;
      case CT::Boolean:  // boolean
        table->PutBoolean(entry.networkTableKey, entry.defaultValue.boolValue);
        break;
      case CT::String:  // string
        table->PutString(entry.networkTableKey, entry.defaultValue.stringValue);
        break;
      default:
        throw std::runtime_error("RestoreDefaults: Unsupported constant type");
    }
  }
}

double NetworkTables::getDoubleValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Double) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue);
}

int NetworkTables::getIntValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Int) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return static_cast<int>(std::round(
      table->GetNumber(entry.networkTableKey, entry.defaultValue.doubleValue)));
}

bool NetworkTables::getBooleanValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Boolean) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetBoolean(entry.networkTableKey, entry.defaultValue.boolValue);
}
std::string NetworkTables::getStringValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::String) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetString(entry.networkTableKey,
                          entry.defaultValue.stringValue);
}

units::velocity::meters_per_second_t NetworkTables::getVelocityValue(
    ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Velocity) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_mps;
}

units::radians_per_second_t NetworkTables::getAngularRateValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::AngularRate) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_rad_per_s;
}

units::meters_per_second_squared_t NetworkTables::getAccelerationValue(
    ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Acceleration) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_mps_sq;
}

units::radians_per_second_squared_t NetworkTables::getAngularAccelerationValue(
    ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::AngularAcceleration) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_rad_per_s_sq;
}

units::time::second_t NetworkTables::getTimeValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Time) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_s;
}

units::current::ampere_t NetworkTables::getCurrentValue(ConstantId id) {
  const ConstantEntry &entry = constantEntries[static_cast<size_t>(id)];
  if (entry.type != CT::Current) {
    throw std::runtime_error("Constant type mismatch for " +
                             entry.networkTableKey);
  }
  return table->GetNumber(entry.networkTableKey,
                          entry.defaultValue.doubleValue) *
         1_A;
}

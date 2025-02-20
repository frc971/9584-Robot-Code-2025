#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTable.h>

#include "Constants.h"

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class NetworkTables {
 public:
  NetworkTables();
  double ControllerVelocityCurveExponent();
  units::velocity::meters_per_second_t MaxSpeed();
  units::radians_per_second_t MaxAngularRate();
  double ControllerRotationCurveExponent();
  double ControllerDeadbandPercentage();
  units::meters_per_second_squared_t SlewTranslateLimit();
  units::radians_per_second_squared_t SlewRotateLimit();
  double RollerMovementForwardVelocity();
  double RollerMovementBackwardVelocity();
  units::time::second_t ArmCoralEjectSequenceWait();
  double ArmCoralEjectPosition();
  double ClimbVelocity();
  double UnclimbVelocity();

  int ClimbButton();
  int UnclimbButton();
  int RollerForwardButton();
  int RollerBackwardButton();
  int ArmUpButton();
  int ArmDownButton();
  void RestoreDefaults();

 private:
  std::shared_ptr<nt::NetworkTable> table;
  const std::string kTableName = "Tuning Constants";
  const std::string kRestoreDefaultsName = "restoreDefaults";
  const std::string kControllerVelocityCurveExponentName =
      "controllerVelocityCurveExponent";
  const std::string kMaxSpeedName = "maxSpeed";
  const std::string kMaxAngularRateName = "maxAngularRate";
  const std::string kControllerRotationCurveExponentName =
      "controllerRotationCurveExponent";
  const std::string kControllerDeadbandPercentageName =
      "controllerDeadbandPercentage";
  const std::string kSlewTranslateLimitName = "slewTranslateLimit";
  const std::string kSlewRotateLimitName = "slewRotateLimit";

  const std::string kRollerMovementForwardVelocityName =
      "rollerMovementForwardVelocity";
  const std::string kRollerMovementBackwardVelocityName =
      "rollerMovementBackwardVelocity";
  const std::string kArmCoralEjectSequenceWaitName =
      "armCoralEjectSequenceWait";
  const std::string kArmCoralEjectPositionName = "armCoralEjectPosition";
  const std::string kClimbVelocityName = "climbVelocity";
  const std::string kUnclimbVelocityName = "unclimbVelocity";

  const std::string kClimbButtonName = "climbButton";
  const std::string kUnclimbButtonName = "unclimbButton";
  const std::string kRollerForwardButtonName = "rollerForwardButton";
  const std::string kRollerBackwardButtonName = "rollerBackwardButton";
  const std::string kArmUpButtonName = "armUpButton";
  const std::string kArmDownButtonName = "armDownButton";

  void InitNumber(std::string name, double number);
  void InitRestoreDefaults();

  nt::BooleanSubscriber resetSub;
  NT_Listener resetListenerHandle;
};
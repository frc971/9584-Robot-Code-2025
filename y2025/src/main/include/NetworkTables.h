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
  int ClimbButton();
  int UnclimbButton();
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
  const std::string kClimbButtonName = "climbButton";
  const std::string kUnclimbButtonName = "unclimbButton";
  void InitNumber(std::string name, double number);
  void InitRestoreDefaults();

  nt::BooleanSubscriber resetSub;
  NT_Listener resetListenerHandle;
};
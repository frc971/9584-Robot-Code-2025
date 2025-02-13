#include "NetworkTables.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <iostream>

NetworkTables::NetworkTables() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  table = inst.GetTable(kTableName);
  InitNumber(kControllerVelocityCurveExponentName,
             DriveConstants::kControllerVelocityCurveExponent);
  InitNumber(kMaxSpeedName, DriveConstants::kMaxSpeed.value());
  InitNumber(kMaxAngularRateName, DriveConstants::kMaxAngularRate.value());
  InitNumber(kControllerRotationCurveExponentName,
             DriveConstants::kControllerRotationCurveExponent);
  InitNumber(kControllerDeadbandPercentageName,
             DriveConstants::kControllerDeadbandPercentage);
  InitNumber(kSlewTranslateLimitName,
             DriveConstants::kSlewTranslateLimit.value());
  InitNumber(kSlewRotateLimitName, DriveConstants::kSlewRotateLimit.value());
  InitNumber(kClimbButtonName, DriveConstants::kClimbButton);

  InitNumber(kUnclimbButtonName, DriveConstants::kUnclimbButton);
}

double NetworkTables::ControllerVelocityCurveExponent() {
  return table->GetNumber(kControllerVelocityCurveExponentName,
                          DriveConstants::kControllerVelocityCurveExponent);
}
units::velocity::meters_per_second_t NetworkTables::MaxSpeed() {
  return table->GetNumber(kMaxSpeedName, DriveConstants::kMaxSpeed.value()) *
         1_mps;
}

units::radians_per_second_t NetworkTables::MaxAngularRate() {
  return table->GetNumber(kMaxAngularRateName,
                          DriveConstants::kMaxAngularRate.value()) *
         1_rad_per_s;
}

double NetworkTables::ControllerRotationCurveExponent() {
  return table->GetNumber(kControllerRotationCurveExponentName,
                          DriveConstants::kControllerRotationCurveExponent);
}

double NetworkTables::ControllerDeadbandPercentage() {
  return table->GetNumber(kControllerDeadbandPercentageName,
                          DriveConstants::kControllerDeadbandPercentage);
}

units::meters_per_second_squared_t NetworkTables::SlewTranslateLimit() {
  return table->GetNumber(kSlewTranslateLimitName,
                          DriveConstants::kSlewTranslateLimit.value()) *
         1_mps_sq;
}

units::radians_per_second_squared_t NetworkTables::SlewRotateLimit() {
  return table->GetNumber(kSlewRotateLimitName,
                          DriveConstants::kSlewRotateLimit.value()) *
         1_rad_per_s_sq;
}

int NetworkTables::ClimbButton() {
  return std::round(
      table->GetNumber(kClimbButtonName, DriveConstants::kClimbButton));
}

int NetworkTables::UnclimbButton() {
  return std::round(
      table->GetNumber(kUnclimbButtonName, DriveConstants::kUnclimbButton));
}

void NetworkTables::InitNumber(std::string name, double number) {
  table->SetDefaultNumber(name, number);
  table->SetPersistent(name);
}
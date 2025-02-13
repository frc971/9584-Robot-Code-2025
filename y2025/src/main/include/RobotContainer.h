// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "NetworkTables.h"
#include "Telemetry.h"
#include "subsystems/Climber.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Intake.h"

class RobotContainer {
 public:
  NetworkTables networkTables;

 private:
  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric fieldCentricDrive =
      swerve::requests::FieldCentric{}
          .WithDeadband(networkTables.MaxSpeed() *
                        networkTables.ControllerDeadbandPercentage())
          .WithRotationalDeadband(networkTables.MaxAngularRate() *
                                  networkTables.ControllerDeadbandPercentage())
          .WithDriveRequestType(
              swerve::DriveRequestType::Velocity);  // Use closed-loop control
                                                    // for drive motors
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};
  swerve::requests::RobotCentric robotCentricDrive =
      swerve::requests::RobotCentric{}.WithDriveRequestType(
          swerve::DriveRequestType::Velocity);  // Use closed-loop control for
                                                // drive motors

  /* Note: This must be constructed before the drivetrain, otherwise we need to
   *       define a destructor to un-register the telemetry from the drivetrain
   */
  Telemetry logger{networkTables.MaxSpeed()};

  frc2::CommandXboxController controller{0};
  frc2::CommandXboxController buttonBoard{1};

 public:
  subsystems::CommandSwerveDrivetrain drivetrain{
      TunerConstants::CreateDrivetrain()};
  Climber climber;
  Intake intake;

 private:
  /* Path follower */
  frc::SendableChooser<frc2::Command *> autoChooser;

 public:
  RobotContainer();
  void RobotInit();

  frc2::CommandPtr GetAutonomousCommand();
  static double ExponentialConvert(double controllerValue, double exponent);

 private:
  void ConfigureBindings();
  frc::SlewRateLimiter<units::meters_per_second> fieldXSlewFilter{
      networkTables.SlewTranslateLimit()};
  frc::SlewRateLimiter<units::meters_per_second> fieldYSlewFilter{
      networkTables.SlewTranslateLimit()};
  frc::SlewRateLimiter<units::radians_per_second> fieldRotateSlewFilter{
      networkTables.SlewRotateLimit()};
  frc::SlewRateLimiter<units::meters_per_second> robotXSlewFilter{
      networkTables.SlewTranslateLimit()};
  frc::SlewRateLimiter<units::meters_per_second> robotYSlewFilter{
      networkTables.SlewTranslateLimit()};
  frc::SlewRateLimiter<units::radians_per_second> robotRotateSlewFilter{
      networkTables.SlewRotateLimit()};
};

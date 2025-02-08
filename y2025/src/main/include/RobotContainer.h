// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "subsystems/Climber.h"
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

class RobotContainer {
private:
    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric fieldCentricDrive = swerve::requests::FieldCentric{}
        .WithDeadband(DriveConstants::kMaxSpeed * DriveConstants::kControllerDeadbandPercentage)
        .WithRotationalDeadband(DriveConstants::kMaxAngularRate * DriveConstants::kControllerDeadbandPercentage)
        .WithDriveRequestType(swerve::DriveRequestType::Velocity); // Use closed-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric robotCentricDrive = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity); // Use closed-loop control for drive motors

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{DriveConstants::kMaxSpeed};

    frc2::CommandXboxController controller{0};
    frc2::CommandXboxController buttonBoard{1};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    Climber climber;
    
private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> autoChooser;

public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    static double ExponentialConvert(double controllerValue, double exponent);

private:
    void ConfigureBindings();
    frc::SlewRateLimiter<units::meters_per_second> fieldXSlewFilter{DriveConstants::kSlewTranslateLimit};
    frc::SlewRateLimiter<units::meters_per_second> fieldYSlewFilter{DriveConstants::kSlewTranslateLimit};
    frc::SlewRateLimiter<units::radians_per_second> fieldRotateSlewFilter{DriveConstants::kSlewRotateLimit};
    frc::SlewRateLimiter<units::meters_per_second> robotXSlewFilter{DriveConstants::kSlewTranslateLimit};
    frc::SlewRateLimiter<units::meters_per_second> robotYSlewFilter{DriveConstants::kSlewTranslateLimit};
    frc::SlewRateLimiter<units::radians_per_second> robotRotateSlewFilter{DriveConstants::kSlewRotateLimit};
};

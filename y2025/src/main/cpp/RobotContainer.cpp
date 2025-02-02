// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include "ctre/phoenix6/swerve/SwerveRequest.hpp"

RobotContainer::RobotContainer()
{
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> ctre::phoenix6::swerve::requests::SwerveRequest& {
            if (!controller.RightBumper().Get()) { // Right bumper not pressed
                wpi::outs() << "Field centric drive\n";
                auto fieldX = fieldXSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetLeftY(), JOYSTICK_CONVERT_EXPONENT_VELOCITY)));
                auto fieldY = fieldYSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetLeftX(), JOYSTICK_CONVERT_EXPONENT_VELOCITY)));
                auto fieldRotate = fieldRotateSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetRightX(), JOYSTICK_CONVERT_EXPONENT_ROTATION)));
                return fieldCentricDrive
                .WithVelocityX(fieldX.value() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(fieldY.value() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(fieldRotate.value() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            } else { // Right bumper pressed
                wpi::outs() << "Robot centric drive\n";
                auto robotX = robotXSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetLeftY(), JOYSTICK_CONVERT_EXPONENT_VELOCITY)));
                auto robotY = robotYSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetLeftX(), JOYSTICK_CONVERT_EXPONENT_VELOCITY)));
                auto robotRotate = robotRotateSlewFilter.Calculate(units::scalar_t(ExponentialConvert(-controller.GetRightX(), JOYSTICK_CONVERT_EXPONENT_ROTATION)));
                return robotCentricDrive
                .WithVelocityX(robotX.value() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(robotY.value() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(robotRotate.value() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            }
        })
    );

    controller.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    controller.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-controller.GetLeftY(), -controller.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (controller.Back() && controller.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (controller.Back() && controller.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (controller.Start() && controller.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (controller.Start() && controller.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    controller.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    // Climb and Unclimb button board assignments
    buttonBoard.Button(CLIMB_BUTTON).OnTrue(climber.Climb());
    buttonBoard.Button(UNCLIMB_BUTTON).OnTrue(climber.Unclimb());

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

double RobotContainer::ExponentialConvert(double controllerValue, double exponent) {
  return copysign(pow(controllerValue, exponent), controllerValue);
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}

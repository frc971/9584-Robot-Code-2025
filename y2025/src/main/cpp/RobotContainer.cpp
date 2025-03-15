// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <wpinet/WebServer.h>

#include <iostream>
#include <memory>

#include "ctre/phoenix6/swerve/SwerveRequest.hpp"
#include "subsystems/Intake.h"

using namespace pathplanner;
using ConstantId = NetworkTables::ConstantId;

RobotContainer::RobotContainer() {
  NamedCommands::registerCommand("Eject Coral",
                                 std::move(autoCommands.EjectCoral()));
  NamedCommands::registerCommand("Intake Algae",
                                 std::move(autoCommands.IntakeAlgae()));
  NamedCommands::registerCommand("Eject Algae",
                                 std::move(autoCommands.EjectAlgae()));

  autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  frc::SmartDashboard::PutData("Restore Defaults",
                               std::make_unique<frc2::InstantCommand>([this] {
                                 std::cout << "Restoring defaults\n";
                                 networkTables->RestoreDefaults();
                               }).release());

  ConfigureBindings();
}

void RobotContainer::RobotInit() {
  intake->RobotInit();
  wpi::WebServer::GetInstance().Start(5800,
                                      frc::filesystem::GetDeployDirectory());
}

void RobotContainer::ConfigureBindings() {
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.ApplyRequest(
          [this]() -> ctre::phoenix6::swerve::requests::SwerveRequest& {
            if (!controller.RightBumper().Get()) {  // Right bumper not pressed
              // Drive forward with negative Y (forward)
              auto fieldX = fieldXSlewFilter.Calculate(
                  networkTables->getVelocityValue(ConstantId::MaxSpeed) *
                  ExponentialConvert(
                      -controller.GetLeftY(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerVelocityCurveExponent)));
              // Drive left with negative X (left)
              auto fieldY = fieldYSlewFilter.Calculate(
                  networkTables->getVelocityValue(ConstantId::MaxSpeed) *
                  ExponentialConvert(
                      -controller.GetLeftX(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerVelocityCurveExponent)));
              // Drive counterclockwise with negative X (left)
              auto fieldRotate = fieldRotateSlewFilter.Calculate(
                  networkTables->getAngularRateValue(
                      ConstantId::MaxAngularRate) *
                  ExponentialConvert(
                      -controller.GetRightX(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerRotationCurveExponent)));
              return fieldCentricDrive.WithVelocityX(fieldX)
                  .WithVelocityY(fieldY)
                  .WithRotationalRate(fieldRotate);
            } else {  // Right bumper pressed
              wpi::outs() << "Robot centric drive\n";
              // Drive forward with negative Y (forward)
              auto robotX = robotXSlewFilter.Calculate(
                  networkTables->getVelocityValue(ConstantId::MaxSpeed) *
                  ExponentialConvert(
                      controller.GetLeftY(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerVelocityCurveExponent)));
              // Drive left with negative X (left)
              auto robotY = robotYSlewFilter.Calculate(
                  networkTables->getVelocityValue(ConstantId::MaxSpeed) *
                  ExponentialConvert(
                      controller.GetLeftX(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerVelocityCurveExponent)));
              // Drive counterclockwise with negative X (left)
              auto robotRotate = robotRotateSlewFilter.Calculate(
                  networkTables->getAngularRateValue(
                      ConstantId::MaxAngularRate) *
                  ExponentialConvert(
                      -controller.GetRightX(),
                      networkTables->getDoubleValue(
                          ConstantId::ControllerRotationCurveExponent)));
              return robotCentricDrive.WithVelocityX(robotX)
                  .WithVelocityY(robotY)
                  .WithRotationalRate(robotRotate);
            }
          }));

  controller.A().WhileTrue(
      drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
  controller.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    return point.WithModuleDirection(
        frc::Rotation2d{-controller.GetLeftY(), -controller.GetLeftX()});
  }));

  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  (controller.Back() && controller.Y())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  (controller.Back() && controller.X())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  (controller.Start() && controller.Y())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (controller.Start() && controller.X())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  controller.LeftBumper().OnTrue(
      drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  // Button board assignments
  buttonBoard.Button(networkTables->getIntValue(ConstantId::ArmUpButton))
      .OnTrue(intake->ArmUpPressed())
      .OnFalse(intake->ArmUpReleased());
  buttonBoard.Button(networkTables->getIntValue(ConstantId::ArmDownButton))
      .OnTrue(intake->ArmDownPressed())
      .OnFalse(intake->ArmDownReleased());
  buttonBoard
      .Button(networkTables->getIntValue(ConstantId::RollerForwardButton))
      .OnTrue(intake->RollerForwardPressed())
      .OnFalse(intake->RollerForwardReleased());
  buttonBoard
      .Button(networkTables->getIntValue(ConstantId::RollerBackwardButton))
      .OnTrue(intake->RollerBackwardPressed())
      .OnFalse(intake->RollerBackwardReleased());
  buttonBoard.Button(networkTables->getIntValue(ConstantId::ClimbButton))
      .OnTrue(climber.ClimbPressed())
      .OnFalse(climber.ClimbReleased());
  buttonBoard.Button(networkTables->getIntValue(ConstantId::UnclimbButton))
      .OnTrue(climber.UnclimbPressed())
      .OnFalse(climber.UnclimbReleased());
  buttonBoard.Button(networkTables->getIntValue(ConstantId::ResetEncoderButton))
      .OnTrue(intake->ResetEncoderPositionCommand());
  buttonBoard
      .AxisGreaterThan(
          networkTables->getIntValue(ConstantId::AlgaeIntakeButtonAxis), 0.75)
      .OnTrue(intake->AlgaeIntakePressed())
      .OnFalse(intake->AlgaeIntakeReleased());
  buttonBoard
      .AxisGreaterThan(
          networkTables->getIntValue(ConstantId::AlgaeEjectButtonAxis), 0.75)
      .OnTrue(intake->AlgaeEjectPressed())
      .OnFalse(intake->AlgaeEjectReleased());
  buttonBoard.POVUp()
      .OnTrue(intake->CoralEjectPressed())
      .OnFalse(intake->CoralEjectReleased());

  for (int i = 1; i < 11; i++) {
    buttonBoard.Button(i).OnTrue(
        frc2::cmd::Print("Button " + std::to_string(i) + " pressed"));
  }
  drivetrain.RegisterTelemetry(
      [this](auto const& state) { logger.Telemeterize(state); });
}

void RobotContainer::TeleopInit() { intake->TeleopInit(); }

void RobotContainer::AutonomousInit() { intake->AutonomousInit(); }

double RobotContainer::ExponentialConvert(double controllerValue,
                                          double exponent) {
  return copysign(pow(abs(controllerValue), exponent), controllerValue);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}

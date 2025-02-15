// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "LimelightHelpers.h"
#include "sim/PhysicsSim.h"

Robot::Robot() {}

void Robot::RobotInit() { m_container.RobotInit(); }

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutNumber(
      "CAN Utilization %",
      frc::RobotController::GetCANStatus().percentBusUtilization * 100.0);
  frc::SmartDashboard::PutNumber(
      "Voltage", frc::RobotController::GetBatteryVoltage().value());
  frc::SmartDashboard::PutNumber("CPU Temperature",
                                 frc::RobotController::GetCPUTemp().value());
  frc::SmartDashboard::PutBoolean("RSL", frc::RobotController::GetRSLState());
  frc::SmartDashboard::PutNumber("Match Time",
                                 frc::DriverStation::GetMatchTime().value());

  frc::SmartDashboard::PutNumber(
      "Code Runtime (ms)", frc::Timer::GetFPGATimestamp().value() * 1000.0);

  /*
   * This example of adding Limelight is very simple and may not be sufficient
   * for on-field use. Users typically need to provide a standard deviation that
   * scales with the distance to target and changes with number of tags
   * available.
   *
   * This example is sufficient to show that vision integration is possible,
   * though exact implementation of how to use vision should be tuned per-robot
   * and to the team's specification.
   */
  if (kUseLimelight) {
    auto const driveState = m_container.drivetrain.GetState();
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;

    LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0,
                                          0, 0);
    auto llMeasurement =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (llMeasurement && llMeasurement->tagCount > 0 &&
        units::math::abs(omega) < 2_tps) {
      m_container.drivetrain.AddVisionMeasurement(
          llMeasurement->pose, llMeasurement->timestampSeconds);
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::SimulationPeriodic() { PhysicsSim::GetInstance().Run(); }

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

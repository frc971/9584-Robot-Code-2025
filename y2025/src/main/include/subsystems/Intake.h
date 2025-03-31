#pragma once

#include <NetworkTables.h>
#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"
#include "subsystems/CommandSwerveDrivetrain.h"

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class Intake : public frc2::SubsystemBase {
 public:
  Intake(std::shared_ptr<NetworkTables> networkTables,
         swerve::requests::RobotCentric robotCentricDrive);

  void RobotInit();
  void TeleopInit();
  void ResetDefaultPosition();
  void PrintPosition();
  void AutonomousInit();
  frc2::CommandPtr AlgaeIntakePressed();
  frc2::CommandPtr AlgaeIntakeReleased();
  frc2::CommandPtr AlgaeEjectPressed();
  frc2::CommandPtr AlgaeEjectReleased();
  frc2::CommandPtr CoralEjectPressed();
  frc2::CommandPtr CoralEjectReleased();
  frc2::CommandPtr RollerForwardPressed();
  frc2::CommandPtr RollerForwardReleased();
  frc2::CommandPtr RollerBackwardPressed();
  frc2::CommandPtr RollerBackwardReleased();
  frc2::CommandPtr ArmUpPressed();
  frc2::CommandPtr ArmUpReleased();
  frc2::CommandPtr ArmDownPressed();
  frc2::CommandPtr ArmDownReleased();
  frc2::CommandPtr ResetEncoderPositionCommand();
  frc2::CommandPtr AutoIntakeCoral();

 private:
  frc::DigitalInput m_coralBeamBreak{0};
  std::shared_ptr<NetworkTables> m_networkTables;
  swerve::requests::RobotCentric m_robotCentricDrive;

  TalonSRX armMotor{16};
  VictorSPX rollerMotor{18};
};

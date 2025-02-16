#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  void RobotInit();
  void TeleopInit();
  frc2::CommandPtr AlgaeIntakePressed();
  frc2::CommandPtr AlgaeIntakeReleased();
  frc2::CommandPtr AlgaeEjectPressed();
  frc2::CommandPtr AlgaeEjectReleased();
  frc2::CommandPtr CoralEjectPressed();
  frc2::CommandPtr CoralEjectReleased();
  void ResetPosition();
  void PrintPosition();

 private:
  TalonSRX armMotor{16};
  VictorSPX rollerMotor{18};
};

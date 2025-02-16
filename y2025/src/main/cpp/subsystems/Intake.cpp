#include "subsystems/Intake.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>

#include "Constants.h"

using namespace frc2::cmd;
using namespace frc2;
using namespace DriveConstants;
using namespace ctre::phoenix::motorcontrol;

Intake::Intake() {}

void Intake::RobotInit() {
  armMotor.ConfigFactoryDefault();
  // Configure PID constants (These values will need tuning!)
  armMotor.Config_kP(0, kArmMotorProportionalGainValue,
                     10);                                   // Proportional gain
  armMotor.Config_kI(0, kArmMotorIntegralGainValue, 10);    // Integral gain
  armMotor.Config_kD(0, kArmMotorDerivativeGainValue, 10);  // Derivative gain
  armMotor.Config_kF(0, kArmMotorFeedForwardGainValue, 10);
  armMotor.SetSelectedSensorPosition(kArmSelectedSensorPosition, 0, 10);
  armMotor.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
  // Set the allowable error in sensor units (ticks)
  armMotor.ConfigAllowableClosedloopError(0, kArmMotorAllowableCloseLoopError,
                                          10);

  // Set the peak and nominal outputs
  armMotor.ConfigNominalOutputForward(kArmMotorForwardNominalPercentOutput, 10);
  armMotor.ConfigNominalOutputReverse(kArmMotorReverseNominalPercentOutput, 10);
  armMotor.ConfigPeakOutputForward(kArmMotorForwardPeakPercentOutput, 10);
  armMotor.ConfigPeakOutputReverse(kArmMotorReversePeakPercentOutput, 10);
  armMotor.ConfigMotionCruiseVelocity(kArmMotorMagicMotionCruiseVelocity, 10);
  armMotor.ConfigMotionAcceleration(kArmMotorMagicMotionAccelerationVelocity,
                                    10);
  // Set sensor phase (may need to be true or false depending on encoder
  // direction)
  armMotor.SetSensorPhase(false);

  armMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Intake::TeleopInit() {
  ResetPosition();
  armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, 0);
  rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
}

CommandPtr Intake::AlgaeIntakePressed() {
  std::cout << "============ AlgaeIntakePressed\n";
  return Sequence(
      RunOnce([this] {
        std::cout << "lowering arm\n";
        std::cout << "Position1: " << armMotor.GetSelectedSensorPosition(0);
        armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                     kArmIntakePosition);
      }),
      Wait(0.2_s), RunOnce([this] {
        std::cout << "stopping the lowering of arm";
        std::cout << "Position2: " << armMotor.GetSelectedSensorPosition(0);
        rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                        kRollerMovementForwardVelocity);
      }));
}

CommandPtr Intake::AlgaeIntakeReleased() {
  std::cout << "============ AlgaeIntakeReleased\n";
  return Sequence(
      RunOnce([this] {
        std::cout << "raising arm\n";
        std::cout << "Position3: " << armMotor.GetSelectedSensorPosition();
        armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                     kArmHoldPosition);
      }),
      Wait(0.2_s), RunOnce([this] {
        std::cout << "stopping the raising of arm";
        std::cout << "Position4: " << armMotor.GetSelectedSensorPosition(0);
        rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
      }));
}

CommandPtr Intake::AlgaeEjectPressed() {
  std::cout << "============ AlgaeEjectPressed\n";
  return RunOnce([this] {
    std::cout << "stopping the ejecting lowering of arm";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    kRollerMovementBackwardVelocity);
  });
}

CommandPtr Intake::AlgaeEjectReleased() {
  std::cout << "============ AlgaeEjectReleased\n";
  return RunOnce([this] {
    std::cout << "raising arm after eject\n";
    armMotor.Set(TalonSRXControlMode::Position, kDefaultPosition);
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

CommandPtr Intake::CoralEjectPressed() {
  std::cout << "============ CoralEjectPressed\n";
  return RunOnce([this] {
    std::cout << "moving rollers forward\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    kRollerMovementForwardVelocity);
  });
}

CommandPtr Intake::CoralEjectReleased() {
  std::cout << "============ CoralEjectReleased\n";
  return RunOnce([this] {
    std::cout << "stopping rollers\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

void Intake::ResetPosition() {
  std::cout << "Reseting position" << std::endl;
  armMotor.SetSelectedSensorPosition(0, 0, 10);
  std::cout << "Position2: " << armMotor.GetSelectedSensorPosition(0)
            << std::endl;
}

void Intake::PrintPosition() {
  std::cout << "Position: " << armMotor.GetSelectedSensorPosition(0)
            << std::endl;
}

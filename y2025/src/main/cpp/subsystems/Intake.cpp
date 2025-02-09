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
  armMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

CommandPtr Intake::AlgaeIntakePressed() {
  std::cout << "============ AlgaeIntakePressed\n";
  return Either(Sequence(RunOnce([this] {
                           std::cout << "lowering arm\n";
                           currentState = State::ALGAE_INTAKE;
                           armMotor.Set(VictorSPXControlMode::PercentOutput,
                                        kArmMovementForwardVelocity);
                         }),
                         Wait(kArmMovementPeriod), RunOnce([this] {
                           std::cout << "stopping the lowering of arm";
                           armMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                           rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                           kRollerMovementForwardVelocity);
                         })),
                None(), [this] { return currentState == State::DEFAULT; });
}

CommandPtr Intake::AlgaeIntakeReleased() {
  std::cout << "============ AlgaeIntakeReleased\n";
  return Either(Sequence(RunOnce([this] {
                           std::cout << "raising arm\n";
                           currentState = State::ALGAE_HOLD;
                           armMotor.Set(VictorSPXControlMode::PercentOutput,
                                        kArmMovementReverseVelocity);
                         }),
                         Wait(kArmMovementHoldPeriod), RunOnce([this] {
                           std::cout << "stopping the raising of arm";
                           armMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                           rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                           0);
                         })),
                None(), [this] { return currentState == State::ALGAE_INTAKE; });
}

CommandPtr Intake::AlgaeEjectPressed() {
  std::cout << "============ AlgaeEjectPressed\n";
  return Either(Sequence(RunOnce([this] {
                           std::cout << "lowering arm to eject\n";
                           currentState = State::ALGAE_EJECT;
                           armMotor.Set(VictorSPXControlMode::PercentOutput,
                                        kArmMovementEjectVelocity);
                         }),
                         Wait(kArmMovementHoldPeriod), RunOnce([this] {
                           std::cout << "stopping the ejecting lowering of arm";
                           armMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                           rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                           kRollerMovementBackwardVelocity);
                         })),
                None(), [this] { return currentState == State::ALGAE_HOLD; });
}

CommandPtr Intake::AlgaeEjectReleased() {
  std::cout << "============ AlgaeEjectReleased\n";
  return Either(Sequence(RunOnce([this] {
                           std::cout << "raising arm after eject\n";
                           currentState = State::DEFAULT;
                           armMotor.Set(VictorSPXControlMode::PercentOutput,
                                        kArmMovementReverseVelocity);
                           rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                           0);
                         }),
                         Wait(kArmMovementPeriod), RunOnce([this] {
                           std::cout << "stopping the ejected raising of arm";
                           armMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                         })),
                None(), [this] { return currentState == State::ALGAE_EJECT; });
}

CommandPtr Intake::CoralEjectPressed() {
  std::cout << "============ CoralEjectPressed\n";
  return Either(RunOnce([this] {
                  std::cout << "moving rollers forward\n";
                  currentState = State::CORAL_EJECT;
                  rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                  kRollerMovementForwardVelocity);
                }),
                None(), [this] { return currentState == State::DEFAULT; });
}

CommandPtr Intake::CoralEjectReleased() {
  std::cout << "============ CoralEjectReleased\n";
  return Either(RunOnce([this] {
                  std::cout << "stopping rollers\n";
                  currentState = State::DEFAULT;
                  rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                }),
                None(), [this] { return currentState == State::CORAL_EJECT; });
}

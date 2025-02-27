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
  rollerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Intake::TeleopInit() {}

void Intake::ResetDefaultPosition() {
  std::cout << "Resetting position" << std::endl;
  armMotor.SetSelectedSensorPosition(0, 0, 10);
  std::cout << "Position2: " << armMotor.GetSelectedSensorPosition(0)
            << std::endl;
  armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
               kArmDefaultPosition);
}

CommandPtr Intake::ResetEncoderPositionCommand() {
  return RunOnce([this] { ResetDefaultPosition(); });
}

void Intake::AutonomousInit() {
  ResetDefaultPosition();
  rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
               DriveConstants::kArmDefaultPosition);
}

void Intake::PrintPosition() {
  std::cout << "Position: " << armMotor.GetSelectedSensorPosition(0)
            << std::endl;
}

CommandPtr Intake::AlgaeIntakePressed() {
  return Sequence(
      RunOnce([this] {
        std::cout << "============ AlgaeIntakePressed\n";
        std::cout << "lowering arm\n";
        std::cout << "Position1: " << armMotor.GetSelectedSensorPosition(0)
                  << std::endl;
        armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                     kArmIntakePosition);
      }),
      Wait(kAlgaeIntakeSequenceWait), RunOnce([this] {
        std::cout << "stopping the lowering of arm\n";
        std::cout << "Position2: " << armMotor.GetSelectedSensorPosition(0)
                  << std::endl;
        rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                        kRollerMovementForwardVelocity);
      }));
}

CommandPtr Intake::AlgaeIntakeReleased() {
  return Sequence(
      RunOnce([this] {
        std::cout << "============ AlgaeIntakeReleased\n";
        std::cout << "raising arm\n";
        std::cout << "Position3: " << armMotor.GetSelectedSensorPosition()
                  << std::endl;
        armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                     kArmHoldPosition);
      }),
      Wait(kAlgaeIntakeSequenceWait), RunOnce([this] {
        std::cout << "stopping the raising of arm";
        std::cout << "Position4: " << armMotor.GetSelectedSensorPosition(0)
                  << std::endl;
        rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                        kRollerMovementHoldVelocity);
      }));
}

CommandPtr Intake::AlgaeEjectPressed() {
  return RunOnce([this] {
    std::cout << "============ AlgaeEjectPressed\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    kRollerMovementBackwardVelocity);
  });
}

CommandPtr Intake::AlgaeEjectReleased() {
  return RunOnce([this] {
    std::cout << "============ AlgaeEjectReleased\n";
    armMotor.Set(TalonSRXControlMode::Position, kArmDefaultPosition);
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

CommandPtr Intake::CoralEjectPressed() {
  return Sequence(RunOnce([this] {
                    std::cout << "============ CoralEjectPressed\n";
                    std::cout << "moving rollers forward\n";
                    armMotor.Set(TalonSRXControlMode::Position,
                                 kArmDefaultPosition);
                    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                                    kRollerMovementForwardVelocity);
                  }),
                  Wait(kArmCoralEjectSequenceWait), RunOnce([this] {
                    std::cout << "lowering of arm";
                    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                    armMotor.Set(TalonSRXControlMode::Position,
                                 kArmCoralEjectPosition);
                  }))
      .FinallyDo(
          [this] { rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0); });
}

CommandPtr Intake::CoralEjectReleased() {
  return RunOnce([this] {
    std::cout << "============ CoralEjectReleased\n";
    std::cout << "Resetting arm position\n";
    armMotor.Set(TalonSRXControlMode::Position, kArmDefaultPosition);
  });
}

CommandPtr Intake::RollerForwardPressed() {
  return RunOnce([this] {
    std::cout << "============ Rollers Forward\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    kRollerMovementForwardVelocity);
  });
}

CommandPtr Intake::RollerForwardReleased() {
  return RunOnce([this] {
    std::cout << "============ Rollers Stopped\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

CommandPtr Intake::RollerBackwardPressed() {
  return RunOnce([this] {
    std::cout << "============ Rollers Backward\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    kRollerMovementBackwardVelocity);
  });
}

CommandPtr Intake::RollerBackwardReleased() {
  return RunOnce([this] {
    std::cout << "============ Rollers Stopped\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

CommandPtr Intake::ArmUpPressed() {
  return RunOnce([this] {
    std::cout << "============ Arm up\n";
    armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                 kArmUpVelocity);
  });
}

CommandPtr Intake::ArmUpReleased() {
  return RunOnce([this] {
    std::cout << "============ Arm stopped\n";
    armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                 armMotor.GetSelectedSensorPosition(0));
  });
}

CommandPtr Intake::ArmDownPressed() {
  return RunOnce([this] {
    std::cout << "============ Arm down\n";
    armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                 kArmDownVelocity);
  });
}

CommandPtr Intake::ArmDownReleased() {
  return RunOnce([this] {
    std::cout << "============ Arm stopped\n";
    armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                 armMotor.GetSelectedSensorPosition(0));
  });
}
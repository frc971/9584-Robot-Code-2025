#include "subsystems/Intake.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>

#include "Constants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace frc2::cmd;
using namespace frc2;
using namespace ctre::phoenix::motorcontrol;
using ConstantId = NetworkTables::ConstantId;

Intake::Intake(std::shared_ptr<NetworkTables> networkTables,
               swerve::requests::RobotCentric robotCentricDrive)
    : m_networkTables(networkTables), m_robotCentricDrive(robotCentricDrive) {}

void Intake::RobotInit() {
  armMotor.ConfigFactoryDefault();
  // Configure PID constants (These values will need tuning!)
  armMotor.Config_kP(0,
                     m_networkTables->getDoubleValue(
                         ConstantId::ArmMotorProportionalGainValue),
                     10);  // Proportional gain
  armMotor.Config_kI(
      0, m_networkTables->getDoubleValue(ConstantId::ArmMotorIntegralGainValue),
      10);  // Integral gain
  armMotor.Config_kD(
      0,
      m_networkTables->getDoubleValue(ConstantId::ArmMotorDerivativeGainValue),
      10);  // Derivative gain
  armMotor.Config_kF(
      0,
      m_networkTables->getDoubleValue(ConstantId::ArmMotorFeedForwardGainValue),
      10);
  armMotor.SetSelectedSensorPosition(
      m_networkTables->getDoubleValue(ConstantId::ArmSelectedSensorPosition), 0,
      10);
  armMotor.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
  // Set the allowable error in sensor units (ticks)
  armMotor.ConfigAllowableClosedloopError(
      0,
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorAllowableCloseLoopError),
      10);

  // Set the peak and nominal outputs
  armMotor.ConfigNominalOutputForward(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorForwardNominalPercentOutput),
      10);
  armMotor.ConfigNominalOutputReverse(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorReverseNominalPercentOutput),
      10);
  armMotor.ConfigPeakOutputForward(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorForwardPeakPercentOutput),
      10);
  armMotor.ConfigPeakOutputReverse(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorReversePeakPercentOutput),
      10);
  armMotor.ConfigMotionCruiseVelocity(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorMagicMotionCruiseVelocity),
      10);
  armMotor.ConfigMotionAcceleration(
      m_networkTables->getDoubleValue(
          ConstantId::ArmMotorMagicMotionAccelerationVelocity),
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
               m_networkTables->getDoubleValue(ConstantId::ArmDefaultPosition));
}

CommandPtr Intake::ResetEncoderPositionCommand() {
  return RunOnce([this] { ResetDefaultPosition(); });
}

void Intake::AutonomousInit() {
  ResetDefaultPosition();
  rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
               m_networkTables->getDoubleValue(ConstantId::ArmDefaultPosition));
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
        armMotor.Set(
            ctre::phoenix::motorcontrol::ControlMode::Position,
            m_networkTables->getDoubleValue(ConstantId::ArmIntakePosition));
      }),
      Wait(m_networkTables->getTimeValue(ConstantId::AlgaeIntakeSequenceWait)),
      RunOnce([this] {
        std::cout << "stopping the lowering of arm\n";
        std::cout << "Position2: " << armMotor.GetSelectedSensorPosition(0)
                  << std::endl;
        rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                        m_networkTables->getDoubleValue(
                            ConstantId::RollerMovementAlgaeIntakeVelocity));
      }));
}

CommandPtr Intake::AlgaeIntakeReleased() {
  return Sequence(
      RunOnce([this] {
        std::cout << "============ AlgaeIntakeReleased\n";
        std::cout << "raising arm\n";
        std::cout << "Position3: " << armMotor.GetSelectedSensorPosition()
                  << std::endl;
        armMotor.Set(
            ctre::phoenix::motorcontrol::ControlMode::Position,
            m_networkTables->getDoubleValue(ConstantId::ArmHoldPosition));
      }),
      Wait(m_networkTables->getTimeValue(ConstantId::AlgaeIntakeSequenceWait)),
      RunOnce([this] {
        std::cout << "stopping the raising of arm";
        std::cout << "Position4: " << armMotor.GetSelectedSensorPosition(0)
                  << std::endl;
        rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                        m_networkTables->getDoubleValue(
                            ConstantId::RollerMovementHoldVelocity));
      }));
}

CommandPtr Intake::AlgaeEjectPressed() {
  return RunOnce([this] {
    std::cout << "============ AlgaeEjectPressed\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    m_networkTables->getDoubleValue(
                        ConstantId::RollerMovementAlgaeEjectVelocity));
  });
}

CommandPtr Intake::AlgaeEjectReleased() {
  return RunOnce([this] {
    std::cout << "============ AlgaeEjectReleased\n";
    armMotor.Set(
        TalonSRXControlMode::Position,
        m_networkTables->getDoubleValue(ConstantId::ArmDefaultPosition));
    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
  });
}

CommandPtr Intake::CoralEjectPressed() {
  return Sequence(RunOnce([this] {
                    std::cout << "============ CoralEjectPressed\n";
                    std::cout << "moving rollers forward\n";
                    armMotor.Set(TalonSRXControlMode::Position,
                                 m_networkTables->getDoubleValue(
                                     ConstantId::ArmDefaultPosition));
                    rollerMotor.Set(
                        VictorSPXControlMode::PercentOutput,
                        m_networkTables->getDoubleValue(
                            ConstantId::RollerMovementCoralEjectVelocity));
                  }),
                  WaitUntil([this] { return !m_coralBeamBreak.Get(); }),
                  RunOnce([this] {
                    std::cout << "lowering of arm";
                    rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0);
                    armMotor.Set(TalonSRXControlMode::Position,
                                 m_networkTables->getDoubleValue(
                                     ConstantId::ArmCoralEjectPosition));
                  }))
      .FinallyDo(
          [this] { rollerMotor.Set(VictorSPXControlMode::PercentOutput, 0); });
}

CommandPtr Intake::CoralEjectReleased() {
  return RunOnce([this] {
    std::cout << "============ CoralEjectReleased\n";
    std::cout << "Resetting arm position\n";
    armMotor.Set(
        TalonSRXControlMode::Position,
        m_networkTables->getDoubleValue(ConstantId::ArmDefaultPosition));
  });
}

CommandPtr Intake::RollerForwardPressed() {
  return RunOnce([this] {
    std::cout << "============ Rollers Forward\n";
    rollerMotor.Set(VictorSPXControlMode::PercentOutput,
                    m_networkTables->getDoubleValue(
                        ConstantId::RollerMovementForwardVelocity));
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
                    m_networkTables->getDoubleValue(
                        ConstantId::RollerMovementBackwardVelocity));
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
                 m_networkTables->getDoubleValue(ConstantId::ArmUpVelocity));
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
                 m_networkTables->getDoubleValue(ConstantId::ArmDownVelocity));
  });
}

CommandPtr Intake::ArmDownReleased() {
  return RunOnce([this] {
    std::cout << "============ Arm stopped\n";
    armMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                 armMotor.GetSelectedSensorPosition(0));
  });
}

CommandPtr Intake::AutoIntakeCoral() {
  return WaitUntil([this] {
    std::cout << "Beambreak value: " << m_coralBeamBreak.Get();
    return m_coralBeamBreak.Get();
  });
}
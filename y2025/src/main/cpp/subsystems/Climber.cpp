#include "subsystems/Climber.h"

#include <frc/RobotBase.h>

#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <iostream>

#include "Constants.h"
#include "NetworkTables.h"
#include "frc2/command/Commands.h"
#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"
#include "units/current.h"

using namespace ctre::phoenix6;
using namespace frc2::cmd;
using ConstantId = NetworkTables::ConstantId;

Climber::Climber(std::shared_ptr<NetworkTables> networkTables)
    : m_networkTables(networkTables) {
  PhysicsSim::GetInstance().AddTalonFX(m_motor, 0.001_kg_sq_m);

  configs::TalonFXConfiguration cfg{};

  /* Configure gear ratio */
  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio =
      125;  // 125 rotor rotations per mechanism rotation

  configs::MotorOutputConfigs &moc = cfg.MotorOutput;
  moc.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  /* Configure Motion Magic */
  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity =
      5_tps;  // 5 (mechanism) rotations per second cruise
  mm.MotionMagicAcceleration =
      10_tr_per_s_sq;  // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel
  mm.MotionMagicJerk = 100_tr_per_s_cu;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0.25;  // Add 0.25 V output to overcome static friction
  slot0.kV = 0.12;  // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.01;  // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 60;    // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0;     // No output for integrated error
  slot0.kD = 0.5;   // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_motor.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName()
              << std::endl;
  }

  if (frc::RobotBase::IsSimulation()) {
    PhysicsSim::GetInstance().AddTalonFX(m_motor, 0.001_kg_sq_m);
  }
}

frc2::CommandPtr Climber::ClimbPressed() {
  return Sequence(

      this->RunOnce([this] {
        maxCurrentGoingUp = 0;
        std::cout << "Climbing" << std::endl;
        m_motor.Set(m_networkTables->getDoubleValue(ConstantId::ClimbVelocity));
      }),
      // Wait for the initial spike in torque current (due to the motor taking
      // in a lot of current to start up) to pass
      Wait(units::time::second_t{0.1}), WaitUntil([this]() -> bool {
        if (std::abs(m_motor.GetTorqueCurrent().GetValue().value()) >
            maxCurrentGoingUp) {
          maxCurrentGoingUp = m_motor.GetTorqueCurrent().GetValue().value();
        }
        std::cout << "Going up. Torque Current: "
                  << m_motor.GetTorqueCurrent().GetValue().value()
                  << " Max current ever: " << maxCurrentGoingUp << std::endl;
        return std::abs(m_motor.GetTorqueCurrent().GetValue().value()) >
               m_networkTables
                   ->getCurrentValue(ConstantId::ClimberTorqueCurrentLimit)
                   .value();
      }),
      this->RunOnce([this] {
        std::cout << "Stopping climb because it is at full extension"
                  << std::endl;
        m_motor.Set(0);
        // Prevent the climber from letting the robot slowly descend after climb
        // is finished
        m_motor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{
            m_motor.GetPosition().GetValue()});
      }));
}

frc2::CommandPtr Climber::ClimbReleased() {
  return this->RunOnce([this] {
    std::cout << "Climbing stopped" << std::endl;
    m_motor.Set(0);
    m_motor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{
        m_motor.GetPosition().GetValue()});
  });
}

frc2::CommandPtr Climber::UnclimbPressed() {
  return this->RunOnce([this] {
    std::cout << "Unclimbing" << std::endl;
    m_motor.Set(m_networkTables->getDoubleValue(ConstantId::UnclimbVelocity));
  });
}

frc2::CommandPtr Climber::UnclimbReleased() {
  return this->RunOnce([this] {
    std::cout << "Unclimbing stopped" << std::endl;
    m_motor.Set(0);
  });
}
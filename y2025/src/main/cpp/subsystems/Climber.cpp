#include "subsystems/Climber.h"

#include <frc/RobotBase.h>

#include <iostream>

#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"

using namespace ctre::phoenix6;

Climber::Climber() {
  PhysicsSim::GetInstance().AddTalonFX(m_motor, 0.001_kg_sq_m);

  configs::TalonFXConfiguration cfg{};

  /* Configure gear ratio */
  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio =
      12.8;  // 12.8 rotor rotations per mechanism rotation

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

frc2::CommandPtr Climber::Climb() {
  return this->RunOnce([this] {
    m_motor.SetControl(m_mmReq.WithPosition(CLIMB_DISTANCE).WithSlot(0));
  });
}

frc2::CommandPtr Climber::Unclimb() {
  return this->RunOnce(
      [this] { m_motor.SetControl(m_mmReq.WithPosition(0_tr).WithSlot(0)); });
}
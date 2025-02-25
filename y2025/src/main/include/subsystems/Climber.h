#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "NetworkTables.h"

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class Climber : public frc2::SubsystemBase {
 public:
  Climber(std::shared_ptr<NetworkTables> networkTables);

  frc2::CommandPtr ClimbPressed();
  frc2::CommandPtr ClimbReleased();
  frc2::CommandPtr UnclimbPressed();
  frc2::CommandPtr UnclimbReleased();

 private:
  double maxCurrentGoingUp = 0;
  double maxCurrentGoingDown = 0;
  ctre::phoenix6::hardware::TalonFX m_motor{13, "rio"};
  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};
  const units::angle::turn_t CLIMB_DISTANCE = 1_tr;
  std::shared_ptr<NetworkTables> m_networkTables;
  ctre::phoenix6::controls::PositionVoltage m_positionReq{0_tr};
};
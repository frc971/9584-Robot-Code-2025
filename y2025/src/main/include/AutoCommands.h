#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "subsystems/Intake.h"

class AutoCommands {
 public:
  AutoCommands(std::shared_ptr<Intake> intake);
  frc2::CommandPtr IntakeAlgae();
  frc2::CommandPtr EjectAlgae();
  frc2::CommandPtr EjectCoral();

 private:
  std::shared_ptr<Intake> m_intake;
};
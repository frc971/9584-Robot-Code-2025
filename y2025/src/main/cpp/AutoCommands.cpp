#include "AutoCommands.h"

#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
using namespace frc2;
using namespace frc2::cmd;

AutoCommands::AutoCommands(std::shared_ptr<Intake> intake) : m_intake(intake) {}

CommandPtr AutoCommands::IntakeAlgae() {
  return Sequence(m_intake->AlgaeIntakePressed(),
                  Wait(DriveConstants::kAutoIntakeAlgaeWait),
                  m_intake->AlgaeIntakeReleased());
}

CommandPtr AutoCommands::EjectAlgae() {
  return Sequence(m_intake->AlgaeEjectPressed(),
                  Wait(DriveConstants::kAutoEjectAlgaeWait),
                  m_intake->AlgaeEjectReleased());
}

CommandPtr AutoCommands::EjectCoral() {
  return Sequence(m_intake->CoralEjectPressed(),
                  Wait(DriveConstants::kAutoEjectCoralWait),
                  m_intake->CoralEjectReleased());
}
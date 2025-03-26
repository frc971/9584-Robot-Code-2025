#include "AutoCommands.h"

#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
using namespace frc2;
using namespace frc2::cmd;
using ConstantId = NetworkTables::ConstantId;

AutoCommands::AutoCommands(std::shared_ptr<Intake> intake,
                           std::shared_ptr<NetworkTables> networkTables)
    : m_intake(intake), m_networkTables(networkTables) {}

CommandPtr AutoCommands::IntakeAlgae() {
  return Sequence(
      m_intake->AlgaeIntakePressed(),
      Wait(m_networkTables->getTimeValue(ConstantId::AutoIntakeAlgaeWait)),
      m_intake->AlgaeIntakeReleased());
}

CommandPtr AutoCommands::EjectAlgae() {
  return Sequence(
      m_intake->AlgaeEjectPressed(),
      Wait(m_networkTables->getTimeValue(ConstantId::AutoEjectAlgaeWait)),
      m_intake->AlgaeEjectReleased());
}

CommandPtr AutoCommands::EjectCoral() {
  return Sequence(
      m_intake->CoralEjectPressed(),
      Wait(m_networkTables->getTimeValue(ConstantId::AutoEjectCoralWait)),
      m_intake->CoralEjectReleased());
}

CommandPtr AutoCommands::IntakeCoral() { return m_intake->AutoIntakeCoral(); }
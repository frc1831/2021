#include "commands/CloseClawCommand.h"
#include "RobotContainer.h"
#include "Constants.h"

CloseClawCommand::CloseClawCommand(ClawSubsystem* claw) : m_claw(claw) {
  SetName("Close Claw");
  AddRequirements({m_claw});
}
void CloseClawCommand::Initialize() {
        m_claw->Close(); 
}

void CloseClawCommand::End(bool interrupted) {
  m_claw->Stop();
}

CloseLeftClawCommand::CloseLeftClawCommand(ClawSubsystem* claw) : m_claw(claw) {
  SetName("Close Left Claw");
  AddRequirements({m_claw});
}
void CloseLeftClawCommand::Initialize() {
        m_claw->CloseLeft(); 
}

void CloseLeftClawCommand::End(bool interrupted) {
  m_claw->Stop();
}


CloseRightClawCommand::CloseRightClawCommand(ClawSubsystem* claw) : m_claw(claw) {
  SetName("Close Right Claw");
  AddRequirements({m_claw});
}
void CloseRightClawCommand::Initialize() {
        m_claw->CloseRight(); 
}

void CloseRightClawCommand::End(bool interrupted) {
  m_claw->Stop();
}

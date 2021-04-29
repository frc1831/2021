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
/*
bool CloseClawCommand::IsFinished() {
    return m_claw->IsGripping();
}
*/
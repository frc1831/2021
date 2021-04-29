#include "commands/StopClawCommand.h"
#include "RobotContainer.h"
#include "Constants.h"

StopClawCommand::StopClawCommand(ClawSubsystem* claw) : m_claw(claw) {
  SetName("Stop Claw");
  AddRequirements({m_claw});
}
void StopClawCommand::Initialize() {
        m_claw->Stop(); 
}

void StopClawCommand::End(bool interrupted) {
  
}

/*
bool StopClawCommand::IsFinished() {
    return m_claw->IsGripping();
}
*/
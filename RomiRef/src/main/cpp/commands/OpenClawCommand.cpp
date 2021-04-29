#include "commands/OpenClawCommand.h"
#include "RobotContainer.h"


OpenClawCommand::OpenClawCommand(ClawSubsystem* claw) : m_claw(claw) {
//:  frc2::CommandHelper<frc2::WaitCommand, OpenClawCommand>{(units::time::second_t) ClawConstants::SERVO_ACTION_TIME}, m_claw(claw) 
   SetName("OpenClaw");
   AddRequirements({m_claw});
}
void OpenClawCommand::Initialize() {
        m_claw->Open(); 
//        frc2::WaitCommand::Initialize();
}

void OpenClawCommand::End(bool interrupted) {
    m_claw->Stop();
}

/*
bool OpenClawCommand::IsFinished() {
      return m_claw->IsGripping();
}
*/
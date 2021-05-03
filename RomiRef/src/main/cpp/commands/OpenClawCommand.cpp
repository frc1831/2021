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

OpenLeftClawCommand::OpenLeftClawCommand(ClawSubsystem* claw) : m_claw(claw) {
//:  frc2::CommandHelper<frc2::WaitCommand, OpenClawCommand>{(units::time::second_t) ClawConstants::SERVO_ACTION_TIME}, m_claw(claw) 
   SetName("OpenLeftClaw");
   AddRequirements({m_claw});
}
void OpenLeftClawCommand::Initialize() {
        m_claw->OpenLeft(); 
//        frc2::WaitCommand::Initialize();
}

void OpenLeftClawCommand::End(bool interrupted) {
    m_claw->Stop();
}

OpenRightClawCommand::OpenRightClawCommand(ClawSubsystem* claw) : m_claw(claw) {
//:  frc2::CommandHelper<frc2::WaitCommand, OpenClawCommand>{(units::time::second_t) ClawConstants::SERVO_ACTION_TIME}, m_claw(claw) 
   SetName("OpenRightClaw");
   AddRequirements({m_claw});
}
void OpenRightClawCommand::Initialize() {
        m_claw->OpenRight(); 
//        frc2::WaitCommand::Initialize();
}

void OpenRightClawCommand::End(bool interrupted) {
    m_claw->Stop();
}

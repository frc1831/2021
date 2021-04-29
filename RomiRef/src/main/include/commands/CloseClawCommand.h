#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

 #include <frc2/command/WaitCommand.h>

#include "subsystems/ClawMechanism.h"

class CloseClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, CloseClawCommand> {
     public:
      explicit CloseClawCommand(ClawSubsystem* claw);
      void Initialize() override;
    
     void End(bool interrupted) override;
    // bool IsFinished() override;

    private:
    ClawSubsystem* m_claw;
};

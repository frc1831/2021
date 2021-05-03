#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc2/command/WaitCommand.h>

#include "subsystems/ClawMechanism.h"

class OpenClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, OpenClawCommand> {
     public:
      explicit OpenClawCommand(ClawSubsystem* claw) ;
      void Initialize() override;
  
      void End(bool interrupted) override;
 //     bool IsFinished() override;

    private:
      ClawSubsystem* m_claw;
};

class OpenLeftClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, OpenLeftClawCommand> {
     public:
      explicit OpenLeftClawCommand(ClawSubsystem* claw) ;
      void Initialize() override;
  
      void End(bool interrupted) override;
 //     bool IsFinished() override;

    private:
      ClawSubsystem* m_claw;
};

class OpenRightClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, OpenRightClawCommand> {
     public:
      explicit OpenRightClawCommand(ClawSubsystem* claw) ;
      void Initialize() override;
  
      void End(bool interrupted) override;
 //     bool IsFinished() override;

    private:
      ClawSubsystem* m_claw;
};

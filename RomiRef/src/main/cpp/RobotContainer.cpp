// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/Button.h>

#include "commands/TeleopArcadeDrive.h"

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Also set default commands here
  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive, [this] { return -m_joy01.GetRawAxis(1); },
      [this] { return (fabs(m_joy01.GetRawAxis(2)) < .20) ? 0.0 : (m_joy01.GetRawAxis(2) < 0.0 ? m_joy01.GetRawAxis(2) + .20 : m_joy01.GetRawAxis(2) - .20 ); }));

frc2::JoystickButton m_b3{&m_joy01, 3};
frc2::JoystickButton m_b4{&m_joy01, 4};

 frc2::JoystickButton m_test{&m_joy01, 11};

m_b3.WhenPressed(OpenClawCommand(&m_claw))
    .WhenReleased(StopClawCommand(&m_claw));
m_b4.WhenPressed(CloseClawCommand(&m_claw))
    .WhenReleased(StopClawCommand(&m_claw));

m_test.WhenPressed(frc2::PrintCommand("Button A Pressed"))
      .WhenReleased(frc2::PrintCommand("Button A Released"));

  // Example of how to use the onboard IO
  m_onboardButtonA.WhenPressed(frc2::PrintCommand("Button A Pressed"))
      .WhenReleased(frc2::PrintCommand("Button A Released"));

  // Setup SmartDashboard options.
  m_chooser.SetDefaultOption("Auto Routine Distance", &m_autoDistance);
  m_chooser.AddOption("Auto Routine Time", &m_autoTime);
  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

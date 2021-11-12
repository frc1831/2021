// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/DriveDistance.h"
#include "commands/TurnTime.h"
#include "commands/TurnDegrees.h"

#include "subsystems/Drivetrain.h"

class AutonomousDistance
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutonomousDistance> {
 public:
  explicit AutonomousDistance(Drivetrain* drive) {
    AddCommands(
        /*
        DriveDistance(0.5, 30_in, drive), TurnTime(0.4, 0.96_s, drive),
        DriveDistance(0.5, 10_in, drive), TurnTime(0.4, 0.96_s, drive),
        DriveDistance(0.5, 10_in, drive), TurnTime(0.4, 0.96_s, drive),
        DriveDistance(0.5, 12_in, drive), TurnTime(0.4, 0.96_s, drive),
        DriveDistance(0.5, 35_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 10_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 10_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 25_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 20_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 15_in, drive), TurnTime(-0.4, 0.96_s, drive),
        DriveDistance(0.5, 60_in, drive));
        
        
        DriveDistance(0.6, 30_in, drive), TurnTime(-0.4, 1.2_s, drive),
        DriveDistance(0.6, 14_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 35_in, drive), TurnDegrees(-0.3, 85_deg, drive),
        DriveDistance(0.6, 16_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 15_in, drive), TurnDegrees(-0.3, 85_deg, drive),
        DriveDistance(0.6, 25_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 20_in, drive), TurnDegrees(-0.3, 85_deg, drive),
        DriveDistance(0.6, 15_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 60_in, drive));
*/
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(0.3, 180_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(-0.3, 90_deg, drive),
        DriveDistance(0.6, 12_in, drive), TurnDegrees(-0.3, 180_deg, drive));
  }
};

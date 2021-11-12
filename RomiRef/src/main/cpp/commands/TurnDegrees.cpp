// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnDegrees.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>

#include <units/math.h>
#include <wpi/math>

#include "Constants.h"

using namespace DriveTrainConstants;

void TurnDegrees::Initialize() {
  // Set motors to stop, read encoder values for starting point
  m_drive->ArcadeDrive(0, 0);
  //m_drive->ResetGyro();
  //m_drive->ResetEncoders();
}

void TurnDegrees::Execute() {
  frc2::PrintCommand("Turning Execute");
    m_drive->ArcadeDrive(0, m_speed);
}

void TurnDegrees::End(bool interrupted) {
  m_drive->ArcadeDrive(0, 0);
  m_drive->ResetEncoders();
}

bool TurnDegrees::IsFinished() {
  // Need to convert distance travelled to degrees. The Standard Romi Chassis
  // found here https://www.pololu.com/category/203/romi-chassis-kits, has a
  // wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
  // or 5.551 inches. We then take into consideration the width of the tires.
  // static auto inchPerDegree = (5.551_in * wpi::math::pi) / 360_deg;

  // Compare distance traveled from start to distance based on degree turn.
 //return (GetLeftEncTurningDistance() >= (.95 * (inchPerDegree * m_angle)) || GetRightEncTurningDistance() >= (.95 * (inchPerDegree * m_angle)));
   return GetAverageTurningDistance() >=  METERPERDEGREE *  (double) m_angle;
}

double TurnDegrees::GetAverageTurningDistance() {
  auto l = units::math::abs(m_drive->GetLeftDistance());
  auto r = units::math::abs(m_drive->GetRightDistance());
  auto x = (l+r) / 2;
  frc2::PrintCommand("GetAverageTurning");
  return (double) x;
}

units::meter_t TurnDegrees::GetLeftEncTurningDistance() {
  auto l = units::math::abs(m_drive->GetLeftDistance());
  
  return l;
}
units::meter_t TurnDegrees::GetRightEncTurningDistance() {
    auto r = units::math::abs(m_drive->GetRightDistance());
  return r;
}

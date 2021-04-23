// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>

// CAN IDs for DriveTrain
const static int FRONT_LEFT = 5;
const static int FRONT_RIGHT = 6;
const static int REAR_LEFT = 7;
const static int REAR_RIGHT = 8;


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

// Locations of the wheels relative to the robot center.
// unit is in meters _m
// X forward from center is > 0
// Y left from center is > 0
frc::Translation2d m_frontLeft{ 0.2667_m, 0.254_m};
frc::Translation2d m_frontRight{ 0.2667_m, -0.254_m};
frc::Translation2d m_rearLeft{ -0.2667_m, 0.254_m};
frc::Translation2d m_rearRight{ -0.2667_m, -0.254_m};

frc::MecanumDriveKinematics m_kinematics{
  m_frontLeft, m_frontRight, m_rearLeft, m_rearRight
};

// Creating the odometry object. 
// frc::MecanumDriveOdometry m_odometry { m_kinematics, GetGyroHeading(), frc::Pose2d{ 5_m, 13.5_m, 0_rad}};

};

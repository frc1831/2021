// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/PIDController.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>


#include "AHRS.h"

// Stick0 Controls
const static int SlowDriveON = 3;
const static int SlowDriveOFF = 4;
const static int FieldModeON = 5;
const static int FieldModeOFF = 6;
const static int ReverseCollector = 2;
const static int ZeroYaw = 11;

// Stick1 Controls
const static int ShootStartShooter = 7;
const static int ShootStartFeeder = 9;
const static int ShootStopFeeder = 10;
const static int ShootStopAll = 11;

// Shooter Values
const static float ShooterPower1 = .20;
const static float SHOOTERTOPPOWER = -.25;
const static float FEEDERPOWER = .55;
const static float COLLECTORPOWER = -.45;

// Joystick Deadzone
const static float DeadZone = .00;


// *********Control
const static int kJoystickChannel0 = 0;  // Left Joystick
const static int kJoystickChannel1 = 1;  // Right Joystick



#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 private:
  Drivetrain* m_driveTrain;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

    // Control
  frc::Joystick m_stickLeft{kJoystickChannel0};
  frc::Joystick m_stickRight{kJoystickChannel1};

 
 public:

  //double rotateToAngleRate = 0.00f;
  
  Robot();

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

  
   
};

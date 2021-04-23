// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    m_mecanum.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

 private:
//  frc::XboxController m_controller{0};
  frc::Joystick m_controller0{0};
  frc::Joystick m_controller1{1};
  Drivetrain m_mecanum;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {

    // Joystick Deadzone
    const static float DeadZone = .20;
    float driveX = 0, driveY = 0, rotate = 0;
	  driveX = m_controller0.GetX();
		driveY = m_controller0.GetY();
		rotate = m_controller1.GetZ();

 	  if (fabs(driveX) < DeadZone) driveX = 0.0;
		if (fabs(driveY) < DeadZone) driveY = 0.0;
		if (fabs(rotate) < DeadZone ) rotate = 0.0;

		// Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            driveX) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = m_yspeedLimiter.Calculate(
                            driveY) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         m_controller1.GetZ()) *
                     Drivetrain::kMaxAngularSpeed;

		frc::SmartDashboard::PutNumber("X Value",(double) xSpeed);
		frc::SmartDashboard::PutNumber("Y Value",(double) ySpeed);
		frc::SmartDashboard::PutNumber("Z Value",(double) rot);
    m_mecanum.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

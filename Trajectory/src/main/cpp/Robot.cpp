// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <time.h>

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/WPILib.h>
#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <frc/trajectory/TrajectoryConfig.h>

Robot::Robot() {
    // 	rotateToAngleRate = 0.00f;
	// try {
	// /***********************************************************************
	//  * navX-MXP:
	//  * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
	//  * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
	//  *
	//  * navX-Micro:
	//  * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
	//  * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
	//  *
	//  * Multiple navX-model devices on a single robot are supported.
	//  ************************************************************************/
	// 	ahrs = new AHRS(frc::SPI::Port::kMXP);
	// 	std::cout << "AHRS Connected" << std::endl;
	// } catch (std::exception& ex ) {
	// 	std::string err_string = "Error instantiating navX MXP:  ";
	// 	err_string += ex.what();
	// 	frc::DriverStation::ReportError(err_string.c_str());
	// }

}

void Robot::RobotInit() {
  m_driveTrain = new Drivetrain();

  frc::TrajectoryConfig *config = new frc::TrajectoryConfig((units::velocity::meters_per_second_t) 3, (units::acceleration::meters_per_second_squared_t) 2);
  //config->SetKinematics(m_driveTrain.get)

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	frc::CameraServer::GetInstance()->StartAutomaticCapture();

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
			frc::SmartDashboard::PutString("Auto", "AutonomousInit");

}
void Robot::AutonomousPeriodic() {
  // Need to update Odometry here.
  m_driveTrain->PeriodicPositionUpdate();

  
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
	float driveX = 0, driveY = 0, rotate = 0;
	bool fieldDrive = false;
	bool precisionDrive = false;
	float gyroValue = 0;
  
  m_driveTrain->SetSafetyOff();

  while (IsOperatorControl() && IsEnabled()) {

    frc::SmartDashboard::PutNumber("Heading: ", m_driveTrain->GetFusedHeading());

    // ********************* BEGIN DRIVE CONTROLS ***********************
    driveX = m_stickRight.GetX();
    driveY = m_stickRight.GetY();
    rotate = m_stickLeft.GetZ();

    if (m_stickRight.GetRawButton(SlowDriveON)) precisionDrive = true;
    if (m_stickRight.GetRawButton(SlowDriveOFF)) precisionDrive = false;

    if (m_stickRight.GetRawButton(FieldModeON)) fieldDrive = true;
    if (m_stickRight.GetRawButton(FieldModeOFF)) fieldDrive = false;

    if( m_stickRight.GetRawButton(ZeroYaw)) m_driveTrain->ZeroYaw();

		if (precisionDrive == true){
			if (fabs(driveX) < DeadZone) driveX = 0.0;
			if (fabs(driveY) < DeadZone) driveY = 0.0;
			if (fabs(rotate) < DeadZone ) rotate = 0.0;
			driveX = -(driveX/2);
			driveY = driveY/2;
			rotate = rotate/3;
			rotate = -rotate;
		} else {
			// Forward and Backward			
			if (fabs(driveY) < DeadZone) driveY = 0.0;
			else if (fabs(driveY) >= (DeadZone + 0.01)) {
		        if (fabs(driveY) < 0.7) {
        			driveY = (fabs(driveY)-(DeadZone + 0.01))*(fabs(driveY)/driveY);
        		}
        		else {
          			driveY = (fabs(driveY)-(DeadZone + 0.01))*(1/driveY);
        		}
      		}
			// Side to Side
			if (fabs(driveX) < DeadZone) driveX = 0.0;
       		else if (fabs(driveX) >= (DeadZone + 0.01)) {
       			if (fabs(driveX) < 0.7) {
       				driveX = (fabs(driveX)-(DeadZone + 0.01))*(fabs(driveX)/driveX);
       			}
       			else {
       				driveX = (fabs(driveX)-(DeadZone + 0.01))*(1/driveX);
       			}
      		}


			if (fabs(rotate) < DeadZone) rotate = 0.0;
			else if (fabs(rotate) >= (DeadZone + 0.01)) {
        		if (fabs(rotate) < 0.7) {
          			rotate = (fabs(rotate)-(DeadZone + 0.01))*(fabs(rotate)/rotate);
        		}
        		else {
          			rotate = (fabs(rotate)-(DeadZone + 0.01))*(1/rotate);
        		}
				
      		}

			driveX = -driveX;
			rotate = -rotate;
		}

        bool rotateToAngle = false;
		double currentRotationRate = 0;
		int POVDirection = m_stickRight.GetPOV();
		if (POVDirection >= 0) {
        //     if (POVDirection == 0) {
        //         m_driveTrain->m_turnController.SetSetpoint(0.0f);
        //         rotateToAngle = true;
        //     } else if (POVDirection == 90) {
		//         m_driveTrain->m_turnController.SetSetpoint(90.0f);
		//         rotateToAngle = true;
		//     } else if (POVDirection == 180) {
		//         m_driveTrain->m_turnController.SetSetpoint(179.9f);
		//         rotateToAngle = true;
		//     } else if (POVDirection == 270) {
		//         m_driveTrain->m_turnController.SetSetpoint(-90.0f);
		//         rotateToAngle = true;
		//     }
		    
		// 	if ( rotateToAngle ) {
		//         m_driveTrain->m_turnController.Enable();
		//         currentRotationRate = m_driveTrain->m_rotateToAngleRate;
		//         frc::SmartDashboard::PutNumber("rotateToAngleRate: ", m_driveTrain->m_rotateToAngleRate);
		//     } else {
		//         m_driveTrain->m_turnController.Disable();
		//         currentRotationRate = rotate;
		//     }
		    
			
		// 	m_driveTrain->SetDriveAndTurn(driveX, driveY, currentRotationRate);
		   m_driveTrain->TurnOnly(POVDirection);
		} else {
	        
	        if (!fieldDrive) gyroValue = 0;

			frc::SmartDashboard::PutNumber("X Value",driveX);
			frc::SmartDashboard::PutNumber("Y Value",driveY);
			frc::SmartDashboard::PutNumber("Z Value",rotate);
			// frc::SmartDashboard::PutNumber("Gyro",gyroValue);
			
			m_driveTrain->SetDriveAndTurn(driveX, driveY, rotate);
	        
		}

  }

  m_driveTrain->SetSafetyOn();
  // Need to update Odometry here.
  m_driveTrain->PeriodicPositionUpdate();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}




#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

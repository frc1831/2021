/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <time.h>

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>

double angle = 0;

Robot::Robot() {

	_CollectorTalon = new WPI_VictorSPX(SpinnerTalon);
	_FeederTalon = new WPI_VictorSPX(FeederTalon);

  	rotateToAngleRate = 0.00f;
	try {
	/***********************************************************************
	 * navX-MXP:
	 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
	 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
	 *
	 * navX-Micro:
	 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
	 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
	 *
	 * Multiple navX-model devices on a single robot are supported.
	 ************************************************************************/
		ahrs = new AHRS(frc::SPI::Port::kMXP);
		std::cout << "AHRS Connected" << std::endl;
	} catch (std::exception& ex ) {
		std::string err_string = "Error instantiating navX MXP:  ";
		err_string += ex.what();
		frc::DriverStation::ReportError(err_string.c_str());
	}

	

	turnController = new frc::PIDController(kP, kI, kD, kF, ahrs, this);
	turnController->SetInputRange(-180.0f,  180.0f);
	turnController->SetOutputRange(-1.0, 1.0);
	turnController->SetAbsoluteTolerance(kToleranceDegrees);
	turnController->SetContinuous(true);

}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


	frc::CameraServer::GetInstance()->StartAutomaticCapture();


//    _FrntRiteSpark.SetInverted(false);
//	  _RearRiteSpark.SetInverted(false);

//    _RearLeftSpark.SetInverted(true);
//    _FrntLeftSpark.SetInverted(true);

 	

    m_robotDrive.SetExpiration(0.1);
   	m_robotDrive.SetSafetyEnabled(false);


	rotateToAngleRate = 0.00f;

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
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
/*  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
*/
/*	float driveX = .25;  // Forward and Reverse  range is -1.0 to 1.0
	float driveY = 0.0;
	float rotate = 0.0;
	float gyroValue= 0.0;

	double maxTime = 3.0;
	frc::Timer driveTimer;


	driveTimer.Reset();
	driveTimer.Start();

	frc::SmartDashboard::PutString("Auto", "LoopStart");

	m_robotDrive.DriveCartesian(driveX, driveY, rotate ,gyroValue);

	while (driveTimer.Get() < maxTime){
		frc::SmartDashboard::PutNumber("Timer", driveTimer.Get());
	}
		
	driveTimer.Stop();
	frc::SmartDashboard::PutString("Auto", "LoopStop");
	// Stop Robot
	m_robotDrive.DriveCartesian(0.0, driveY, rotate ,gyroValue);
*/

}

void Robot::TeleopInit() {
   // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  //if (m_autonomousCommand != nullptr) {
  //  m_autonomousCommand.Cancel();
  //  m_autonomousCommand = nullptr;
  //}
}

void Robot::TeleopPeriodic() {

   //frc::Scheduler::GetInstance()->Run(); 
	float driveX = 0, driveY = 0, rotate = 0;
	bool fieldDrive = false;
	bool precisionDrive = false;
	float gyroValue = 0;

	bool bPrevLeftSensor = _LeftSensor.Get();
	bool bPrevRiteSensor = _RiteSensor.Get();
	bool bFeedMotorA = false;
	bool bFeedMotorB = false;
	bool bFeedMotorMstr = false;
	bool bBallDetectedLeft = false;
	bool bBallDetectedRite = false;

	bool bShootStartedShooter = false;  // Used to block access to starting feeder
	bool bShootStartedFeeder = false;

	
	std::string gameData;
	std::string colorDisplayText;

   	m_robotDrive.SetSafetyEnabled(false);
	_FrntRiteEncoder.SetPosition(0);

  	while (IsOperatorControl() && IsEnabled()) {

		frc::SmartDashboard::PutNumber("Heading: ", ahrs->GetFusedHeading());

		// ********************* BEGIN DRIVE CONTROLS ***********************
		driveX = m_stick0.GetX();
		driveY = m_stick0.GetY();
		rotate = m_stick1.GetZ();

		if (m_stick0.GetRawButton(SlowDriveON)) precisionDrive = true;
		if (m_stick0.GetRawButton(SlowDriveOFF)) precisionDrive = false;

		
		
		/* Encoder position is read from a CANEncoder object by calling the
     	* GetPosition() method.
     	* 
     	* GetPosition() returns the position of the encoder in units of revolutions
     	*/
      	frc::SmartDashboard::PutNumber("Encoder Position", _FrntRiteEncoder.GetPosition());
		/**
		 * Encoder velocity is read from a CANEncoder object by calling the
		 * GetVelocity() method.
		 * 
		 * GetVelocity() returns the velocity of the encoder in units of RPM
		 */
		frc::SmartDashboard::PutNumber("Encoder Velocity", _FrntRiteEncoder.GetVelocity());
      

		if (precisionDrive == true){
			if (fabs(driveX) < DeadZone) driveX = 0.0;
			if (fabs(driveY) < DeadZone) driveY = 0.0;
			if (fabs(rotate) < DeadZone) rotate = 0.0;
			driveX = -(driveX/3);
			driveY = driveY/3;
			rotate = rotate/3;

			m_robotDrive.DriveCartesian(driveX, driveY, rotate ,0);
		}
		else {
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

		if (m_stick0.GetRawButton(FieldModeON)) fieldDrive = true;
		if (m_stick0.GetRawButton(FieldModeOFF)) fieldDrive = false;

      	if( m_stick0.GetRawButton(ZeroYaw)) ahrs->ZeroYaw();

        bool rotateToAngle = false;
		double currentRotationRate = 0;
		int POVDirection = m_stick0.GetPOV();
		if (POVDirection >= 0) {
            if (POVDirection == 0) {
                turnController->SetSetpoint(0.0f);
                rotateToAngle = true;
            } else if (POVDirection == 90) {
		        turnController->SetSetpoint(90.0f);
		        rotateToAngle = true;
		    } else if (POVDirection == 180) {
		        turnController->SetSetpoint(179.9f);
		        rotateToAngle = true;
		    } else if (POVDirection == 270) {
		        turnController->SetSetpoint(-90.0f);
		        rotateToAngle = true;
		    }
		    
			if ( rotateToAngle ) {
		        turnController->Enable();
		        currentRotationRate = rotateToAngleRate;
		        frc::SmartDashboard::PutNumber("rotateToAngleRate: ", rotateToAngleRate);
		    } else {
		        turnController->Disable();
		        currentRotationRate = rotate;
		    }
		    
			m_robotDrive.DriveCartesian(driveX, driveY, currentRotationRate ,ahrs->GetAngle());
		} else {
	        gyroValue = ahrs->GetAngle();
	        if (!fieldDrive) gyroValue = 0;

			frc::SmartDashboard::PutNumber("X Value",driveX);
			frc::SmartDashboard::PutNumber("Y Value",driveY);
			frc::SmartDashboard::PutNumber("Z Value",rotate);
			frc::SmartDashboard::PutNumber("Gyro",gyroValue);
	        m_robotDrive.DriveCartesian(driveX, driveY, rotate ,gyroValue);
		}
		// *********************  END DRIVE CONTROLS ***********************

		// **************  BEGIN COLLECTOR/SHOOTER CONTROLS ****************

/*	
		if(_LeftSensor.Get() != bPrevLeftSensor)
		{
			bPrevLeftSensor = !bPrevLeftSensor;
			if(bBallDetectedLeft){
				bBallDetectedLeft = false;
			} else {
				bBallDetectedLeft = true;
			}
		}

		if(_RiteSensor.Get() != bPrevRiteSensor)
		{
			bPrevRiteSensor = !bPrevRiteSensor;
			if(bBallDetectedRite){
				bBallDetectedRite = false;
			} else {
				bBallDetectedRite = true;
			}
		}

		bFeedMotorMstr = bBallDetectedLeft && bBallDetectedRite;

		SmartDashboard::PutBoolean("FeedMotor", bFeedMotorMstr);
		SmartDashboard::PutBoolean("Left Sensor: ",bBallDetectedLeft);
		SmartDashboard::PutBoolean("Rite Sensor: ",bBallDetectedRite);
*/
		// Operate Collector
		if (m_stick0.GetTrigger()) 
		{
			_CollectorTalon->Set(ControlMode::PercentOutput, CollectorPower);
			_FeederTalon->Set(ControlMode::PercentOutput, FeederPower);
		}	
		else
		{
			_CollectorTalon->Set(ControlMode::PercentOutput, 0);
			if(!bShootStartedFeeder)
				_FeederTalon->Set(ControlMode::PercentOutput, 0);
		}

		// Reverse Collector
/*		if (m_stick0.GetRawButton(ReverseCollector))
		{
			_CollectorTalon->Set(ControlMode::PercentOutput, -CollectorPower);
			_FeederTalon->Set(ControlMode::PercentOutput, -FeederPower);
		}	
		else
		{
			_CollectorTalon->Set(ControlMode::PercentOutput, 0);
			if(!bShootStartedFeeder)
				_FeederTalon->Set(ControlMode::PercentOutput, 0);
		}
*/
		
		if (bShootStartedShooter)
		{
			if (m_stick1.GetRawButton(ShootStartFeeder)) 
			{
				_FeederTalon->Set(ControlMode::PercentOutput, FeederPower);
				bShootStartedFeeder = true;
			}
		}

			
		if (m_stick1.GetRawButton(ShootStartShooter))
		{
			bShootStartedShooter = true;
			_ShooterSpark.Set(ShooterPower);
		} 

		frc::SmartDashboard::PutNumber("Encoder Position", _ShooterEncoder.GetPosition());
		frc::SmartDashboard::PutNumber("Shooter RPM", _ShooterEncoder.GetVelocity());

		if(m_stick1.GetRawButton(ShootStopFeeder))
		{
			bShootStartedFeeder = false;
			_FeederTalon->Set(ControlMode::PercentOutput, 0);
		}

		if (m_stick1.GetRawButton(ShootStopAll))
		{
			bShootStartedShooter = false;
			bShootStartedFeeder = false;
			_ShooterSpark.Set(0);
			_FeederTalon->Set(ControlMode::PercentOutput, 0);
		}

		// ***************  END COLLECTOR/SHOOTER CONTROLS *****************

		// ********************  STAGE 3 COLOR CHECK ***********************

		
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(gameData.length() > 0)
		{
			switch (gameData[0])
			{
				case 'B':
					colorDisplayText = "BLUE";
					break;
				case 'G':
					colorDisplayText = "GREEN";
					break;
				case 'R':
					colorDisplayText = "RED";
					break;
				case 'Y':
					colorDisplayText = "YELLOW";
					break;
				default:
					colorDisplayText = "No Color Yet!";
					break;	
			}
		} else {
			colorDisplayText = "No Color Yet!";
		}
		frc::SmartDashboard::PutString("Color for Wheel: ", colorDisplayText);


		// ********************  STAGE 3 COLOR CHECK ***********************
        frc::SmartDashboard::UpdateValues();
        //std::Wait(0.005); // wait 5ms to avoid hogging CPU cycles

  }  // while

  m_robotDrive.SetSafetyEnabled(true);

}

void Robot::TestPeriodic() {

 
}

/* This function is invoked periodically by the PID Controller,
   based upon navX MXP yaw angle input and PID Coefficients.    */
void Robot::PIDWrite(double output) {
    rotateToAngleRate = output;
}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

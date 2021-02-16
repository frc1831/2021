/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>


#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Drive/MecanumDrive.h>
#include <frc/Joystick.h>
#include <frc/PIDController.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMVictorSPX.h>
#include <AHRS.h>

// *******CAN IDs
// Mecanum SPARK Max
//const static int FrntLeftSpark = 5;
//const static int FrntRiteSpark = 6;
//const static int RearLeftSpark = 7;
//const static int RearRiteSpark = 8;
const static int FRONT_LEFT = 5;
const static int FRONT_RIGHT = 6;
const static int REAR_LEFT = 7;
const static int REAR_RIGHT = 8;

const static int ShooterSpark = 9;

// Talon CAN ID'S
const static int SpareTalon1 = 10;
const static int SpareTalon2 = 11;
const static int FeederTalon = 12;
const static int SpinnerTalon = 13;
const static int CollectorTalon = 14;
const static int SpareSPXTalon = 15;

// Other CAN devices
const static int PDP = 32;

// DigitalIO IDs
const static int LeftAlignmentSensor = 8;
const static int RiteAlignmentSensor = 9;

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
const static float ShooterPower = .65;
const static float FeederPower = .80;
const static float CollectorPower = .99;

// Joystick Deadzone
const static float DeadZone = .30;


// *********Control
const static int kJoystickChannel0 = 0;
const static int kJoystickChannel1 = 1;

// *********PID stuff
/* The following PID Controller coefficients will need to be tuned */
/* to match the dynamics of your drive system.  Note that the      */
/* SmartDashboard in Test mode has support for helping you tune    */
/* controllers by displaying a form where you can enter new P, I,  */
/* and D constants and test the mechanism.                         */

const static double kP = 0.03f;
const static double kI = 0.00f;
const static double kD = 0.01f;
const static double kF = 0.00f;

// Encoder Info
const static double kDistancePerRevolution = 25.13;   // 8 Inch Mecanum wheel in inches

// Pulses per revolution   20 pulses per channel per revolution and 4 states (80) x xmission ratio (16:1)  (80x16=1280)
const static double kPulsesPerRevolution = 1280.00;
const static double kDistancePerPulse = 0.19633;   //kDistancePerRevolution / kPulsesPerRevolution;

/* This tuning parameter indicates how close to "on target" the    */
/* PID Controller will attempt to get.                             */
const static double kToleranceDegrees = 2.00f;



class Robot : public frc::TimedRobot, public frc::PIDOutput {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;


  virtual void PIDWrite(double output);
  double rotateToAngleRate = 0.00f;
 
 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  //frc::Command* m_autonomousCommand = nullptr;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

WPI_TalonFX* _FrntLeft = new WPI_TalonFX(FRONT_LEFT);
WPI_TalonFX* _FrntRite = new WPI_TalonFX(FRONT_RIGHT);
WPI_TalonFX* _RearLeft = new WPI_TalonFX(REAR_LEFT);
WPI_TalonFX* _RearRite = new WPI_TalonFX(REAR_RIGHT);



//  rev::CANSparkMax _FrntLeftSpark {FrntLeftSpark, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//  rev::CANSparkMax _FrntRiteSpark {FrntRiteSpark, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//  rev::CANSparkMax _RearLeftSpark {RearLeftSpark, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//  rev::CANSparkMax _RearRiteSpark {RearRiteSpark, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

// NOTE: Not sure why the one below was used.
//  rev::CANEncoder _FrntRiteEncoder = rev::CANEncoder(_FrntRiteSpark, rev::CANEncoder::EncoderType::kHallSensor);

  rev::CANSparkMax _ShooterSpark {ShooterSpark, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANEncoder  _ShooterEncoder = _ShooterSpark.GetEncoder();

	WPI_VictorSPX *_CollectorTalon;
  WPI_VictorSPX *_FeederTalon;

  

	// line sensors
	frc::DigitalInput _LeftSensor {LeftAlignmentSensor};
	frc::DigitalInput _RiteSensor {RiteAlignmentSensor};


/**
 * In RobotInit(), we will configure m_LRSpeedController and m_RRSpeedController to follow 
 * m_LFSpeedController and m_RFSpeedController, respectively. 
 * 
 * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
 * sent to them will automatically be copied by the follower motors
 */
 
//frc::MecanumDrive *m_robotDrive{*_FrntLeft, *_RearLeft, _FrntRite, _RearRite};  
frc::MecanumDrive *m_robotDrive = new frc::MecanumDrive(*_FrntLeft, *_RearLeft, *_FrntRite, *_RearRite);



  /**
 * In order to read encoder values an encoder object is created using the 
 * GetEncoder() method from an existing CANSparkMax object
 */
rev::CANEncoder* _FrntLeftEnc;
rev::CANEncoder* _FrntRiteEnc;

// 	rev::CANEncoder _FrntLeftEnc = _FrntLeftSpark.GetEncoder();
//	rev::CANEncoder _FrntRiteEnc = _FrntRiteSpark.GetEncoder();

  // Control
  frc::Joystick m_stick0{kJoystickChannel0};
  frc::Joystick m_stick1{kJoystickChannel1};
  

	// Navx
    AHRS *ahrs;                         // navX MXP
    frc::PIDController *turnController;      // PID Controller



};

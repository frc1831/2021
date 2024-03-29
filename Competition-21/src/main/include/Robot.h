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
//#include <rev/Rev2mDistanceSensor.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

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

// Non Driving CAN ID'S
const static int COLLECTOR = 15;
const static int SHOOTERBOTTOM = 16;
const static int FEEDER = 17;
const static int SHOOTERTOP = 18;

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
const static float ShooterPower1 = .20;
const static float SHOOTERTOPPOWER = -.25;
const static float FEEDERPOWER = .55;
const static float COLLECTORPOWER = -.45;

// Joystick Deadzone
const static float DeadZone = .00;


// *********Control
const static int kJoystickChannel0 = 0;  // Left Joystick
const static int kJoystickChannel1 = 1;  // Right Joystick

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

static constexpr auto i2cPort = frc::I2C::Port::kOnboard;


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

  rev::CANSparkMax _Collector {COLLECTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax _Feeder {FEEDER, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::CANSparkMax _ShooterBottom {SHOOTERBOTTOM, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::CANSparkMax _ShooterTop {SHOOTERTOP, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANEncoder    _ShooterTopEncoder = _ShooterTop.GetEncoder();

// 2M Distance Sensor
 //  rev::Rev2mDistanceSensor _DistanceSensor {rev::Rev2mDistanceSensor::Port::kOnboard, rev::Rev2mDistanceSensor::DistanceUnit::kInches, rev::Rev2mDistanceSensor::RangeProfile::kLongRange};

// Color Sensor v3
  rev::ColorSensorV3 m_colorSensor{i2cPort};
    /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;
    /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  //  rev::Rev2mDistanceSensor *distSensor;
	// WPI_VictorSPX *_CollectorTalon;
  // WPI_VictorSPX *_FeederTalon;

  

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
  frc::Joystick m_stickLeft{kJoystickChannel0};
  frc::Joystick m_stickRight{kJoystickChannel1};
  

	// Navx
    AHRS *ahrs;                         // navX MXP
    frc::PIDController *turnController;      // PID Controller



};

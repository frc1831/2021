#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/Drive/MecanumDrive.h>
#include <ctre/Phoenix.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <wpi/math>
#include "AHRS.h"


// Motor CANIds
const static int FRONT_LEFT = 5;
const static int FRONT_RIGHT = 6;
const static int REAR_LEFT = 7;
const static int REAR_RIGHT = 8;

constexpr auto STATIC_GAIN = 0.268;
constexpr auto VELOCITY_GAIN = 1.89;
constexpr auto ACCELERATION_GAIN = 0.243;

/************* FOR NAVX *******************************/
/* The following PID Controller coefficients will need to be tuned */
/* to match the dynamics of your drive system.  Note that the      */
/* SmartDashboard in Test mode has support for helping you tune    */
/* controllers by displaying a form where you can enter new P, I,  */
/* and D constants and test the mechanism.                         */

const static double kP = 0.03f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;

/* This tuning parameter indicates how close to "on target" the    */
/* PID Controller will attempt to get.                             */

const static double kToleranceDegrees = 2.0f;


class Drivetrain {
    private: 
        frc::AnalogGyro m_gyro{0};

        WPI_TalonFX m_frontLeftMotor{FRONT_LEFT};
        WPI_TalonFX m_frontRightMotor{FRONT_RIGHT};
        WPI_TalonFX m_backLeftMotor{REAR_LEFT};
        WPI_TalonFX m_backRightMotor{REAR_RIGHT};
            
        // X and Y distance (in meters) from center of robot.
        frc::Translation2d m_frontLeftLocation{ 0.2667_m, 0.254_m};
        frc::Translation2d m_frontRightLocation{ 0.2667_m, -0.254_m};
        frc::Translation2d m_backLeftLocation{ -0.2667_m, 0.254_m};
        frc::Translation2d m_backRightLocation{ -0.2667_m, -0.254_m};

        frc::Encoder m_frontLeftEncoder{0, 1};
        frc::Encoder m_frontRightEncoder{2, 3};
        frc::Encoder m_backLeftEncoder{4, 5};
        frc::Encoder m_backRightEncoder{6, 7};

        frc::MecanumDriveKinematics m_kinematics{
                m_frontLeftLocation, 
                m_frontRightLocation, 
                m_backLeftLocation,
                m_backRightLocation};

        frc::MecanumDriveOdometry m_odometry{m_kinematics, getHeading()};

        frc::Pose2d m_Position;

        //Characterization works in conjunction with PID
        frc::SimpleMotorFeedforward<units::meters> m_feedForward; //{STATIC_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN};

        // Proportional is what is useful for velocity.  Use characterization tool to determine starting P value
        frc2::PIDController m_frontLeftPIDController{1.0, 0.0, 0.0};
        frc2::PIDController m_frontRightPIDController{1.0, 0.0, 0.0};
        frc2::PIDController m_backLeftPIDController{1.0, 0.0, 0.0};
        frc2::PIDController m_backRightPIDController{1.0, 0.0, 0.0};

        frc::MecanumDrive m_MecanumDrive{
            m_frontLeftMotor, 
            m_frontRightMotor, 
            m_backLeftMotor, 
            m_backRightMotor};
        
        AHRS m_navX{SPI::Port::kMXP};

        frc2::PIDController m_turnController{kP, kI, kD};
        double m_rotateToAngleRate;           // Current rotation rate
        double m_currentRotationRate;
    public:
        
        Drivetrain();

        units::degree_t getHeading();

        frc::MecanumDriveWheelSpeeds getWheelSpeeds();

        frc::MecanumDriveKinematics getKinematics();

        void PeriodicPositionUpdate();

        frc::SimpleMotorFeedforward<units::meters> getFeedForward();

        // NavX stuff
        void InitNavX();

        void ZeroYaw();
        float GetFusedHeading();

        // Mecanum calls for Robot

        void SetSafetyOn();
        void SetSafetyOff();

        void SetDriveAndTurn(float driveX, float driveY, double currentRotationRate); 

        void TurnOnly(int degrees);


        void PIDWrite(double output);



};
#include "Drivetrain.h"

#include <frc/controller/PIDController.h>
#include <frc/AnalogGyro.h>






Drivetrain::Drivetrain() {
    
    m_gyro.Reset();
    m_rotateToAngleRate = 0.0f;
    m_currentRotationRate = 0;

}

units::degree_t Drivetrain::getHeading() {
    return (units::degree_t) (-m_gyro.GetAngle());
}

frc::MecanumDriveWheelSpeeds Drivetrain::getWheelSpeeds() {

    // Note:  We will probably need to convert from rotation of motor to speed of wheel

    // getVelocity() / 7.29 * 2 * Math.PI + Units.inchesToMeters(3.0) / 60  (wheel speed in meters/second)

    frc::MecanumDriveWheelSpeeds _wheelSpeeds{
        units::meters_per_second_t(m_frontLeftEncoder.GetRate()),
        units::meters_per_second_t(m_frontRightEncoder.GetRate()),
        units::meters_per_second_t(m_backLeftEncoder.GetRate()),
        units::meters_per_second_t(m_backRightEncoder.GetRate())};
    return _wheelSpeeds;
}

frc::SimpleMotorFeedforward<units::meters> Drivetrain::getFeedForward() {
    return m_feedForward;
}

void Drivetrain::PeriodicPositionUpdate() {
    m_Position = m_odometry.Update(getHeading(), getWheelSpeeds());
}

void Drivetrain::InitNavX() {
    m_navX.Reset();

    //m_turnController.SetInputRange(-180.0f,  180.0f);
    //m_turnController.SetOutputRange(-1.0, 1.0);
    //m_turnController.SetAbsoluteTolerance(kToleranceDegrees);
    //m_turnController.SetContinuous(true);
}

void Drivetrain::ZeroYaw() {
    m_navX.ZeroYaw();
}

float Drivetrain::GetFusedHeading() {
    m_navX.GetFusedHeading();
}

void Drivetrain::SetDriveAndTurn(float driveX, float driveY, double currentRotationRate) {
   m_MecanumDrive.DriveCartesian(driveX, driveY, currentRotationRate ,m_navX.GetAngle());
}

void Drivetrain::TurnOnly(int degrees){
    bool bRotateToAngle = true;

    if (degrees == 0) {
        m_turnController.SetSetpoint(0.0f);
    } else if (degrees == 90) {
        m_turnController.SetSetpoint(90.0f);
    } else if (degrees == 180) {
        m_turnController.SetSetpoint(179.9f);
    } else if (degrees == 270) {
        m_turnController.SetSetpoint(-90.0f);
    }
    else {
        bRotateToAngle = false;
    }
            
		    
    if ( bRotateToAngle ) {
        // m_turnController.Enable();
        m_currentRotationRate = m_rotateToAngleRate;
        SetDriveAndTurn(0.0, 0.0, m_currentRotationRate);
    }
}

void Drivetrain::SetSafetyOff() {
  m_MecanumDrive.SetSafetyEnabled(false);
}

void Drivetrain::SetSafetyOn() {
   m_MecanumDrive.SetSafetyEnabled(true);
}
/* This function is invoked periodically by the PID Controller,
   based upon navX MXP yaw angle input and PID Coefficients.    */
void Drivetrain::PIDWrite(double output) {
    m_rotateToAngleRate = output;
}




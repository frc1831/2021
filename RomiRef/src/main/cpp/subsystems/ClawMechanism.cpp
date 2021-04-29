#include "subsystems/ClawMechanism.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace ClawConstants;

   ClawSubsystem::ClawSubsystem() {
       SetName("Claw");
       AddChild("Servo", &m_servo);
    }

    void ClawSubsystem::Open() {
        m_servo.Set(1.0);
    }

    void ClawSubsystem::Close() {
        m_servo.Set(-1.0);
    }

    void ClawSubsystem::Stop() {
        m_servo.Set(0);
    }

/*
    bool ClawSubsystem::IsGripping() {
        return m_contact.Get();
    }
*/
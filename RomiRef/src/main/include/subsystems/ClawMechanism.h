#pragma once

#include <frc/DigitalInput.h>
#include <frc/Spark.h>

#include <frc2/command/SubsystemBase.h>

//#include <frc/Servo.h>
#include "Constants.h"

class ClawSubsystem :public  frc2::SubsystemBase {
    
public: 
    ClawSubsystem();

    void Open();
    void OpenLeft();
    void OpenRight();


    void Close();
    void CloseLeft();
    void CloseRight();
    

    void Stop();

 //   bool IsGripping();

private:
     frc::Spark m_servo{2};     // PWM number of EXT 3 
     frc::Spark m_servo2{3};     // PWM number of EXT 3 
 //    frc::DigitalInput m_contact{3};  // EXT number
};

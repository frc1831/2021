#pragma once

#include <frc/DigitalInput.h>
#include <frc/Spark.h>

#include <frc2/command/SubsystemBase.h>

//#include <frc/Servo.h>
#include "Constants.h"

using namespace ExternalConstants;

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
     frc::Spark m_servo{EXT_3_PWM};     // PWM number of EXT 3 
     frc::Spark m_servo2{EXT_4_PWM};     // PWM number of EXT 3 
  //   frc::DigitalOutput m_contact{ DIO_PORT};  // EXT number
};

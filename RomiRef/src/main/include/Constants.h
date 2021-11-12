// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// inchPerDegree = (5.551_in * wpi::math::pi) / 360_deg;

// meterPerDegree = .140 * PI / 360 degree

namespace DriveTrainConstants {
    constexpr double METERPERDEGREE = 0.0012; // meters per degree
}

namespace ExternalConstants {
        constexpr int EXT_0_DIO = 8;
        constexpr int EXT_1_ANL_IN = 0;
        constexpr int EXT_2_ANL_IN = 1;
        constexpr int EXT_3_PWM = 2;
        constexpr int EXT_4_PWM = 3;
        
        constexpr int SERVO_PORT = 8;
        constexpr int SERVO_CLOSE = -1;
        constexpr int SERVO_STOP= 0;
        constexpr int SERVO_OPEN = 1;
        constexpr double SERVO_ACTION_TIME = 10.0;
    }

namespace DriveConstants {
constexpr double kCountsPerRevolution = 1440.0;
constexpr double kWheelDiameterInch = 2.75;
}  // namespace DriveConstants

namespace RobotControls {
    constexpr int JOYSTICK01 = 0;
}

// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "communications/ButtonPacket.hpp"
#include "communications/POVPacket.hpp"
#include "communications/PublishNode.hpp"
#include "logging/LogFileSink.hpp"
#include "logging/Logger.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"

namespace frc3512 {

class Robot : public frc::TimedRobot, public PublishNode {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

    static Climber climber;
    static Drivetrain drivetrain;
    static Elevator elevator;
    static Logger logger;

    static frc::Joystick driveStick1;
    static frc::Joystick driveStick2;
    static frc::Joystick appendageStick;

private:
    LogFileSink fileSink{"/home/lvuser/Robot.log"};
};

}  // namespace frc3512

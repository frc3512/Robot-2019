// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cscore.h>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "communications/ButtonPacket.hpp"
#include "communications/CommandPacket.hpp"
#include "communications/POVPacket.hpp"
#include "communications/PublishNode.hpp"
#include "logging/LogFileSink.hpp"
#include "logging/Logger.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/FourBarLift.hpp"
#include "subsystems/Intake.hpp"

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

    static frc::PowerDistributionPanel pdp;

private:
    Climber m_climber;
    Drivetrain m_drivetrain;
    Elevator m_elevator;
    Logger m_logger;
    Intake m_intake;
    FourBarLift m_fourBarLift;

    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStickPort};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};

    LogFileSink fileSink{"/home/lvuser/Robot.log"};

    cs::UsbCamera camera{"Camera 1", 0};
    cs::MjpegServer server{"Server", kMjpegServerPort};
};

}  // namespace frc3512

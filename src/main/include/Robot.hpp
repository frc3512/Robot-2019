// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cscore.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/FourBarLift.hpp"
#include "subsystems/Intake.hpp"

namespace frc3512 {

using namespace frc3512::Constants::Robot;

enum class State {
    kInit,
    kThirdLevel,
    kSecondLevel,
    kFourBarDescend,
    kDescend,
    kDriveForward,
    kIdle
};

class Robot : public RealTimeRobot {
public:
    Robot();

    /**
     * Returns true if currently climbing.
     */
    bool IsClimbing() const;

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;

private:
    Climber m_climber;
    Drivetrain m_drivetrain;
    Elevator m_elevator;
    Intake m_intake;
    FourBarLift m_fourBarLift;

    State m_state = State::kInit;
    bool m_thirdLevel = true;

    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStick1Port};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};

    cs::UsbCamera camera{"Camera 1", 0};
    cs::MjpegServer server{"Server", kMjpegServerPort};

    std::mutex m_cacheMutex;
};

}  // namespace frc3512

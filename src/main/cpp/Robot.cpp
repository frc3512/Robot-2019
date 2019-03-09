// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

#include <frc/DriverStation.h>

using namespace frc3512;

frc::PowerDistributionPanel Robot::pdp;

Robot::Robot() : PublishNode("Robot") {
    m_logger.AddLogSink(fileSink);
    m_logger.Subscribe(m_climber);
    m_logger.Subscribe(m_drivetrain);
    m_logger.Subscribe(m_elevator);
    m_logger.Subscribe(m_intake);
    m_logger.Subscribe(m_fourBarLift);
    m_logger.Subscribe(*this);

    m_climber.Subscribe(*this);
    m_drivetrain.Subscribe(*this);
    m_elevator.Subscribe(*this);
    m_intake.Subscribe(*this);
    m_fourBarLift.Subscribe(*this);
}

void Robot::DisabledInit() {
    CommandPacket message{"DisabledInit", false};
    Publish(message);
}

void Robot::AutonomousInit() {
    CommandPacket message{"AutonomousInit", false};
    Publish(message);
}

void Robot::TeleopInit() {
    CommandPacket message{"TeleopInit", false};
    Publish(message);
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    for (int i = 1; i <= 12; i++) {
        if (m_driveStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick2", i, true};
            Publish(message);
        }
        if (m_appendageStick.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick", i, true};
            Publish(message);
        }
        if (m_appendageStick.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick", i, false};
            Publish(message);
        }
    }
    if (m_appendageStick.GetPOV() == 0) {
        POVPacket message{"AppendagePOV", 0};
    } else if (m_appendageStick.GetPOV() == 180) {
        POVPacket message{"AppendagePOV", 180};
        // TODO: integrate this into Publish/Subscribe system
    } else {
        POVPacket message{"AppendagePOV", -1};
    }
    auto& ds = frc::DriverStation::GetInstance();
    HIDPacket message{"",
                      m_driveStick1.GetX(),
                      m_driveStick1.GetY(),
                      ds.GetStickButtons(0),
                      m_driveStick2.GetX(),
                      m_driveStick2.GetY(),
                      ds.GetStickButtons(1),
                      m_appendageStick.GetX(),
                      m_appendageStick.GetY(),
                      ds.GetStickButtons(2)};
    Publish(message);
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

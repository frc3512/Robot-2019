// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <string>

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

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
    m_climber.Subscribe(m_climber);
    m_climber.Subscribe(m_elevator);
    m_climber.Subscribe(m_fourBarLift);
    m_drivetrain.Subscribe(*this);
    m_elevator.Subscribe(*this);
    m_elevator.Subscribe(m_climber);
    m_elevator.Subscribe(m_fourBarLift);
    m_intake.Subscribe(*this);
    m_fourBarLift.Subscribe(*this);
    m_fourBarLift.Subscribe(m_elevator);
    m_fourBarLift.Subscribe(m_climber);

    camera.SetResolution(160, 120);
    camera.SetFPS(15);
    server.SetSource(camera);

    m_fourBarLift.Subscribe(m_climber);
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
        if (m_appendageStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick2", i, true};
            wpi::outs() << "h\n";
            Publish(message);
        }
        if (m_appendageStick2.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick2", i, false};
            wpi::outs() << "r\n";
            Publish(message);
        }
    }
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousPeriodic() { TeleopPeriodic(); }

void Robot::TeleopPeriodic() {
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
        if (m_appendageStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick2", i, true};
            Publish(message);
        }
        if (m_appendageStick2.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick2", i, false};
            Publish(message);
        }
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
                      ds.GetStickButtons(2),
                      m_appendageStick2.GetX(),
                      m_appendageStick2.GetY(),
                      ds.GetStickButtons(3)};
    Publish(message);
}

void Robot::DisabledPeriodic() {
    wpi::outs() << "FourBar: " << m_fourBarLift.GetHeight() << "\n";
    wpi::outs() << "Elevator: " << m_elevator.GetHeight() << "\n";
    wpi::outs() << "Climber: " << m_climber.GetHeight() << "\n";
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

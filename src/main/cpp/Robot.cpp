// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

using namespace frc3512;

Climber Robot::climber;
CsvLogger Robot::csvLogger{kCSVFile, "Time"};
Drivetrain Robot::drivetrain;
Elevator Robot::elevator;
Logger Robot::logger;

frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};
frc::Joystick Robot::appendageStick{kAppendageStickPort};

Robot::Robot() : PublishNode("Robot") {
    logger.AddLogSink(fileSink);

    climber.Subscribe(*this);
    drivetrain.Subscribe(*this);
    logger.Subscribe(*this);
    elevator.Subscribe(*this);
}

void Robot::DisabledInit() {
    CommandPacket message{"DisabledInit", false};
    Publish(message);
    elevator.Reset();
    elevator.Disable();
}

void Robot::AutonomousInit() {
    elevator.Reset();
    elevator.Enable();
}

void Robot::TeleopInit() {
    CommandPacket message{"TeleopInit", false};
    Publish(message);
    elevator.Reset();
    elevator.Enable();
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    for (int i = 1; i <= 12; i++) {
        if (driveStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick2", i, true};
            Publish(message);
        }
        if (appendageStick.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick", i, true};
            Publish(message);
        }
        if (appendageStick.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick", i, false};
            Publish(message);
        }
    }
    if (appendageStick.GetPOV() == 0) {
        POVPacket message{"AppendagePOV", 0};
    } else if (appendageStick.GetPOV() == 180) {
        POVPacket message{"AppendagePOV", 180};
        // TODO: integrate this into Publish/Subscribe system
    } else {
        POVPacket message{"AppendagePOV", -1};
    }
}

void Robot::DisabledPeriodic() {
    std::cout << elevator.GetHeight() << std::endl;
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

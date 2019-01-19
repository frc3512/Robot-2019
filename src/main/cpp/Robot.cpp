// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

Drivetrain Robot::robotDrive;
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

Robot::Robot() {}

void Robot::DisabledInit() {}

void Robot::AutonomousInit() {}

void Robot::TeleopInit() {}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    std::cout << "LeftDisplacement: " << robotDrive.GetLeftDisplacement()
              << std::endl;

    std::cout << "RightDisplacement: " << robotDrive.GetRightDisplacement()
              << std::endl;
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopPeriodic() {
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(-driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }
    if (driveStick2.GetRawButtonPressed(1)) {
        robotDrive.Shift();
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

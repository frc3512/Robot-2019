// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "Constants.hpp"
#include "Subsystems/Drivetrain.hpp"

class Robot : public frc::TimedRobot {
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

    static Drivetrain robotDrive;
};

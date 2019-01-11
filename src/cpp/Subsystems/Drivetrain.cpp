// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

Drivetrain::Drivetrain() {}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

;

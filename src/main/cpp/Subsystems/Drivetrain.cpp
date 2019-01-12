// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "Robot.hpp"

Drivetrain::Drivetrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

void Drivetrain::Shift() { m_shifter.Set(!m_shifter.Get()); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

void Drivetrain::Debug() {}

double Drivetrain::GetLeftDisplacement() { return m_leftEncoder.Get(); }

double Drivetrain::GetRightDisplacement() { return m_rightEncoder.Get(); }

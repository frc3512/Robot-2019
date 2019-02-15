// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "Robot.hpp"

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_leftGrbx.SetInverted(true);
    m_drive.SetRightSideInverted(false);
    ShiftUp();
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

void Drivetrain::ShiftUp() { m_shifter.Set(true); }

void Drivetrain::ShiftDown() { m_shifter.Set(false); }

void Drivetrain::Shift() { m_shifter.Set(!m_shifter.Get()); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

void Drivetrain::Debug() {}

double Drivetrain::GetLeftDisplacement() { return m_leftEncoder.Get(); }

double Drivetrain::GetRightDisplacement() { return m_rightEncoder.Get(); }

void Drivetrain::SubsystemPeriodic() {
    if (Robot::driveStick1.GetRawButton(1)) {
        Drive(-Robot::driveStick1.GetY() * 0.5, Robot::driveStick2.GetX() * 0.5,
              Robot::driveStick2.GetRawButton(2));
    } else {
        Drive(-Robot::driveStick1.GetY(), Robot::driveStick2.GetX(),
              Robot::driveStick2.GetRawButton(2));
    }
}

void Drivetrain::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/DriveStick2" && message.button == 1 &&
        message.pressed) {
        Shift();
    }
}

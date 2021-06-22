// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/Joystick.h>

using namespace frc3512;
using namespace frc3512::Constants::Intake;

void Intake::SetMotors(MotorState motorState) {
    if (motorState == MotorState::kIntake) {
        m_leftMotor.Set(-0.75);
        m_rightMotor.Set(0.75);
    } else if (motorState == MotorState::kOuttake) {
        m_leftMotor.Set(0.95);
        m_rightMotor.Set(-0.95);
    } else {
        m_leftMotor.Set(0.0);
        m_rightMotor.Set(0.0);
    }
}

void Intake::ToggleClaw() { m_claw.Set(!m_claw.Get()); }

void Intake::TeleopPeriodic() {
    static frc::Joystick appendageStick2{
        Constants::Robot::kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(4)) {
        SetMotors(MotorState::kOuttake);
    } else if (appendageStick2.GetRawButtonPressed(6)) {
        SetMotors(MotorState::kIntake);
    } else if (appendageStick2.GetRawButtonReleased(4)) {
        SetMotors(MotorState::kIdle);
    } else if (appendageStick2.GetRawButtonReleased(6)) {
        SetMotors(MotorState::kIdle);
    } else if (appendageStick2.GetRawButtonPressed(1)) {
        ToggleClaw();
    }
}

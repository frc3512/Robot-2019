// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

using namespace frc3512;

Intake::Intake() : PublishNode("Intake") {}

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

void Intake::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 4 &&
        message.pressed) {
        SetMotors(MotorState::kOuttake);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 6 &&
        message.pressed) {
        SetMotors(MotorState::kIntake);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 4 &&
        !message.pressed) {
        SetMotors(MotorState::kIdle);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 6 &&
        !message.pressed) {
        SetMotors(MotorState::kIdle);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 1 &&
        message.pressed) {
        ToggleClaw();
    }  // TODO get state and put in DS
}

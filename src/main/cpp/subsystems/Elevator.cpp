// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <frc/DigitalInput.h>

#include "Robot.hpp"

using namespace frc3512;

Elevator::Elevator() : PublishNode("Elevator") {}

void Elevator::SetVelocity(double velocity) {
    if (velocity > 0 && m_topLimitSwitch.Get() == m_limitPressedState) {
        velocity = 0.0;
    }
    if (velocity < 0 && m_bottomLimitSwitch.Get() == m_limitPressedState) {
        velocity = 0.0;
    }
    m_grbx.Set(velocity);
}

void Elevator::ResetEncoder() { m_encoder.Reset(); }

double Elevator::GetHeight() { return m_encoder.GetDistance(); }

void Elevator::SubsystemPeriodic() {
    SetVelocity(Robot::appendageStick.GetY());
    if (m_bottomLimitSwitch.Get()) {
        m_encoder.Reset();
    }
}

void Elevator::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    }
}

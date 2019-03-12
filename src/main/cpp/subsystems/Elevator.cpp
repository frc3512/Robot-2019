// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <limits>

#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>

#include "Robot.hpp"

using namespace frc3512;

Elevator::Elevator() : PublishNode("Elevator") {
    m_grbx.Set(0.0);
    m_encoder.SetDistancePerPulse(kElevatorDpP);
    EnablePeriodic();
}

void Elevator::SetVoltage(double voltage) {
    if (voltage > 0.0 && m_topLimitSwitch.Get() == m_limitPressedState) {
        voltage = 0.0;
    }
    if (voltage < 0.0 && m_bottomLimitSwitch.Get() == m_limitPressedState) {
        voltage = 0.0;
    }
    if (voltage > 0.0 && m_encoder.GetDistance() >= kElevatorMax) {
        voltage = 0.0;
    }
    m_grbx.Set(voltage);
}

void Elevator::ResetEncoder() { m_encoder.Reset(); }

double Elevator::GetHeight() { return m_encoder.GetDistance(); }

double Elevator::GetVelocity() {
    static double last = m_encoder.GetRate();

    // TODO make average A more accurate std::cout << (m_encoder.GetRate() -
    // last) / 0.02 << std::endl;
    last = m_encoder.GetRate();
    return 0;
}

bool Elevator::GetMagneticSwitch() {
    return m_bottomLimitSwitch.Get() == m_limitPressedState;
}

void Elevator::Enable() {
    m_controller.Enable();
    m_thread.StartPeriodic(0.005);
}

void Elevator::Disable() {
    m_controller.Disable();
    m_thread.Stop();
}

void Elevator::SetGoal(double position) { m_controller.SetGoal(position); }

bool Elevator::AtReference() const { return m_controller.AtReferences(); }

void Elevator::Iterate() {
    m_controller.SetMeasuredPosition(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_grbx.Set(m_controller.ControllerVoltage() / batteryVoltage);
}

double Elevator::ControllerVoltage() const {
    return m_controller.ControllerVoltage();
}

void Elevator::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

void Elevator::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 12 &&
        message.pressed) {
        SetGoal(1.32);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 1 &&
        message.pressed) {
        SetGoal(0.0);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 11 &&
        message.pressed) {
        SetGoal(0.5);
    }
}

void Elevator::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        Enable();
    }
    if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Enable();
    }
    if (message.topic == "Robot/DisabledInit" && !message.reply) {
        Disable();
    }
}

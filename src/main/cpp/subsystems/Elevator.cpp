// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <chrono>
#include <limits>

#include <frc/DriverStation.h>

#include "Robot.hpp"

using namespace frc3512;
using namespace std::chrono_literals;

Elevator::Elevator() : PublishNode("Elevator") {
    m_grbx.Set(0.0);
    m_encoder.SetDistancePerPulse(kElevatorDpP);
    EnablePeriodic();
}

void Elevator::SetVoltage(double voltage) { m_grbx.Set(voltage); }

void Elevator::ResetEncoder() { m_encoder.Reset(); }

double Elevator::GetHeight() { return m_encoder.GetDistance(); }

void Elevator::Enable() {
    m_controller.Enable();
    m_thread.StartPeriodic(0.005);
    m_isEnabled = true;
}

void Elevator::Disable() {
    m_controller.Disable();
    m_thread.Stop();
    m_isEnabled = false;
}

void Elevator::SetScoringIndex() { m_controller.SetScoringIndex(); }

void Elevator::SetClimbingIndex() { m_controller.SetClimbingIndex(); }

void Elevator::SetGoal(double position) { m_controller.SetGoal(position); }

bool Elevator::AtReference() const { return m_controller.AtReferences(); }

bool Elevator::AtGoal() { return m_controller.AtGoal(); }

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

void Elevator::SubsystemPeriodic() {
    ElevatorStatusPacket message{
        "", m_encoder.GetDistance(), m_controller.ControllerVoltage(),
        m_controller.AtReferences(), m_controller.AtGoal()};
    Publish(message);
}

void Elevator::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 3 &&
        message.pressed) {
        SetGoal(kFloorHeight);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 11 &&
        message.pressed) {
        SetGoal(kBottomHatch);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 12 &&
        message.pressed) {
        SetGoal(kBottomCargo);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 9 &&
        message.pressed) {
        SetGoal(kMiddleHatch);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 10 &&
        message.pressed) {
        SetGoal(kMiddleCargo);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 7 &&
        message.pressed) {
        SetGoal(kTopHatch);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 8 &&
        message.pressed) {
        SetGoal(kTopCargo);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 2 &&
        message.pressed) {
        SetGoal(kCargoShip);
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
    if (message.topic == "Climber/ThirdLevel") {
        SetGoal(kHab3);
    }
    if (message.topic == "Climber/SecondLevel") {
        SetGoal(kHab2);
    }
    if (message.topic == "Climber/ClimbingProfile") {
        m_controller.SetClimbingIndex();
    }
    if (message.topic == "Climber/Down3" || message.topic == "Climber/Down2") {
        SetGoal(0);
    }
    if (message.topic == "Climber/ScoringProfile") {
        m_controller.SetScoringIndex();
    }
}

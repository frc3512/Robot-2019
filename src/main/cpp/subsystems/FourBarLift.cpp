// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/FourBarLift.hpp"

#include <chrono>
#include <limits>

#include <frc/DriverStation.h>

#include "Robot.hpp"

using namespace frc3512;
using namespace std::chrono_literals;

FourBarLift::FourBarLift() : PublishNode("FourBarLift") {
    m_grbx.Set(0.0);
    m_encoder.SetDistancePerPulse(kFourBarLiftDpP);
    EnablePeriodic();
    m_grbx.SetInverted(true);
    SetGoal(0.0);
}

void FourBarLift::SetVoltage(double voltage) { m_grbx.Set(voltage); }

void FourBarLift::ResetEncoder() { m_encoder.Reset(); }

double FourBarLift::GetHeight() { return m_encoder.GetDistance(); }

void FourBarLift::Enable() {
    m_controller.Enable();
    m_thread.StartPeriodic(0.005);
}

void FourBarLift::Disable() {
    m_controller.Disable();
    m_thread.Stop();
}

void FourBarLift::SetGoal(double position) { m_controller.SetGoal(position); }

bool FourBarLift::AtReference() const { return m_controller.AtReferences(); }

bool FourBarLift::AtGoal() { return m_controller.AtGoal(); }

void FourBarLift::Iterate() {
    m_controller.SetMeasuredAngle(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_grbx.Set(m_controller.ControllerVoltage() / batteryVoltage);
}

void FourBarLift::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

void FourBarLift::SubsystemPeriodic() {
    FourBarLiftStatusPacket message{
        "", m_encoder.GetDistance(), m_controller.ControllerVoltage(),
        m_controller.AtReferences(), m_controller.AtGoal()};
    Publish(message);
}

void FourBarLift::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 3 &&
        message.pressed) {
        SetGoal(kFourBarLiftMin);
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 2 &&
        message.pressed) {
        SetGoal(kFourBarLiftMax);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 11 &&
        message.pressed) {
        SetGoal(kFourBarBottomHatch);
    }
    if (message.topic == "Robot/DriveStick2" && message.button == 7 &&
        message.pressed) {
        m_controller.SetClimbing(true);
    }
    if (message.topic == "Robot/DriveStick2" && message.button == 8 &&
        message.pressed) {
        m_controller.SetClimbing(false);
    }
    if (message.topic == "Robot/AppendageStick" && message.pressed) {
        if (message.button == 12 || message.button == 9 ||
            message.button == 10 || message.button == 7 ||
            message.button == 8) {
            SetGoal(kFourBarLiftMax);
        }
    }
}

void FourBarLift::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        Enable();
    }
    if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Enable();
    }
    if (message.topic == "Robot/DisabledInit" && !message.reply) {
        Disable();
    }
    if (message.topic == "Climber/FourBarStart") {
        m_controller.SetClimbing(true);
        SetGoal(-1.704);
    }
    if (message.topic == "Climber/Up") {
        m_controller.SetClimbing(false);
        SetGoal(0);
    }
}

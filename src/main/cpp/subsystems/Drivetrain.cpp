// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <frc/DriverStation.h>

using namespace frc3512;
using namespace frc3512::Constants::Drivetrain;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_leftGrbx.SetInverted(true);
    m_drive.SetRightSideInverted(false);
    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);
    ShiftUp();
    EnablePeriodic();
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

void Drivetrain::ShiftUp() { m_shifter.Set(true); }

void Drivetrain::ShiftDown() { m_shifter.Set(false); }

void Drivetrain::Shift() { m_shifter.Set(!m_shifter.Get()); }

double Drivetrain::GetAngle() const { return m_gyro.GetAngle(); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

double Drivetrain::GetLeftDisplacement() const { return m_leftEncoder.Get(); }

double Drivetrain::GetRightDisplacement() const { return m_rightEncoder.Get(); }

double Drivetrain::GetLeftRate() const { return m_leftEncoder.GetRate(); }

double Drivetrain::GetRightRate() const { return m_rightEncoder.GetRate(); }

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::EnableController() {
    m_controllerThread.StartPeriodic(Constants::kDt.to<double>());
    m_controller.Enable();
    m_drive.SetSafetyEnabled(false);
}

void Drivetrain::DisableController() {
    m_controllerThread.Stop();
    m_controller.Disable();
    m_drive.SetSafetyEnabled(true);
}

bool Drivetrain::IsControllerEnabled() const {
    return m_controller.IsEnabled();
}

void Drivetrain::Reset() {
    m_controller.Reset();
    ResetEncoders();
    ResetGyro();
}

void Drivetrain::Iterate() {
    m_controller.SetMeasuredStates(GetLeftRate(), GetRightRate(), GetAngle());
    m_controller.Update();

    // Set motor inputs
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    SetLeftManual(m_controller.ControllerLeftVoltage() / batteryVoltage);
    SetRightManual(m_controller.ControllerRightVoltage() / batteryVoltage);
}

void Drivetrain::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/DriveStick2" && message.button == 1 &&
        message.pressed) {
        Shift();
    }
}

void Drivetrain::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    }
}

void Drivetrain::ProcessMessage(const HIDPacket& message) {
    if (!IsControllerEnabled()) {
        if (GetRawButton(message, 0, 1)) {
            Drive(-message.y1 * 0.5, message.x2 * 0.5,
                  GetRawButton(message, 1, 2));
        } else {
            Drive(-message.y1, message.x2, GetRawButton(message, 1, 2));
        }
    }
}

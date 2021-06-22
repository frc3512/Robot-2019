// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <chrono>

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

using namespace frc3512;
using namespace frc3512::Constants::Climber;

Climber::Climber()
    : ControlledSubsystemBase("Climber",
                              {ControllerLabel{"Position", "m"},
                               ControllerLabel{"Velocity", "m/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Position", "m"}}) {
    m_encoder.SetReverseDirection(true);
    m_encoder.SetDistancePerPulse(kDpP);
    m_lift.SetInverted(true);
    m_timer.Start();
}

void Climber::SetDriveVoltage(double voltage) { m_drive.Set(voltage); }

void Climber::SetVoltage(double voltage) { m_lift.Set(voltage); }

void Climber::ResetEncoder() { m_encoder.Reset(); }

double Climber::GetHeight() { return m_encoder.GetDistance(); }

double Climber::GetVelocity() { return m_encoder.GetRate(); }

void Climber::ControllerPeriodic() {
    m_controller.SetMeasuredPosition(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();

    SetVoltage(m_controller.ControllerVoltage() / batteryVoltage);
}

void Climber::AutonomousInit() { SetGoal(0.02); }

void Climber::TeleopInit() { SetGoal(0.02); }

void Climber::SetGoal(double position) { m_controller.SetGoal(position); }

bool Climber::AtReference() const { return m_controller.AtReferences(); }

bool Climber::AtGoal() const { return m_controller.AtGoal(); }

double Climber::ControllerVoltage() { return m_controller.ControllerVoltage(); }

void Climber::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

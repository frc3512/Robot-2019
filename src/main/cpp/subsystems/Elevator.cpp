// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <chrono>
#include <limits>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

using namespace frc3512;
using namespace frc3512::Constants::Elevator;
using namespace std::chrono_literals;

Elevator::Elevator()
    : ControlledSubsystemBase("Elevator",
                              {ControllerLabel{"Position", "m"},
                               ControllerLabel{"Velocity", "m/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Position", "m"}}) {
    m_grbx.SetSmartCurrentLimit(60);
    m_grbx.SetInverted(true);
    m_grbx.Set(0.0);
    Reset();
    m_encoder.SetDistancePerPulse(kDpP);
}

void Elevator::SetVoltage(double voltage) { m_grbx.Set(voltage); }

void Elevator::ResetEncoder() { m_encoder.Reset(); }

double Elevator::GetHeight() { return m_encoder.GetDistance(); }

void Elevator::SetScoringIndex() { m_controller.SetScoringIndex(); }

void Elevator::SetClimbingIndex() { m_controller.SetClimbingIndex(); }

void Elevator::SetGoal(double position) { m_controller.SetGoal(position); }

bool Elevator::AtReference() const { return m_controller.AtReferences(); }

bool Elevator::AtGoal() { return m_controller.AtGoal(); }

double Elevator::ControllerVoltage() const {
    return m_controller.ControllerVoltage();
}

void Elevator::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

void Elevator::ControllerPeriodic() {
    UpdateDt();

    m_controller.SetMeasuredPosition(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_grbx.Set(m_controller.ControllerVoltage() / batteryVoltage);

    // Log(m_controller.GetReferences(), )

    if constexpr (frc::RobotBase::IsSimulation()) {
        if (m_controller.IsClimbing()) {
            m_elevatorClimbingSim.SetInput(frc::MakeMatrix<1, 1>(
                m_grbx.Get() * frc::RobotController::GetInputVoltage()));
            m_elevatorClimbingSim.Update(GetDt());
            m_encoderSim.SetDistance(
                m_elevatorClimbingSim.GetPosition().to<double>());
            // SimBattery estimates loaded battery voltages
            frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
                {m_elevatorClimbingSim.GetCurrentDraw()}));
        } else {
            m_elevatorScoringSim.SetInput(frc::MakeMatrix<1, 1>(
                m_grbx.Get() * frc::RobotController::GetInputVoltage()));
            m_elevatorScoringSim.Update(GetDt());
            m_encoderSim.SetDistance(
                m_elevatorScoringSim.GetPosition().to<double>());
            // SimBattery estimates loaded battery voltages
            frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
                {m_elevatorClimbingSim.GetCurrentDraw()}));
        }
    }
}

void Elevator::TeleopPeriodic() {
    static frc::Joystick appendageStick2{
        Constants::Robot::kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(3)) {
        SetGoal(kFloorHeight);
    } else if (appendageStick2.GetRawButtonPressed(11)) {
        SetGoal(kBottomHatch);
    } else if (appendageStick2.GetRawButtonPressed(12)) {
        SetGoal(kBottomCargo);
    } else if (appendageStick2.GetRawButtonPressed(9)) {
        SetGoal(kMiddleHatch);
    } else if (appendageStick2.GetRawButtonPressed(10)) {
        SetGoal(kMiddleCargo);
    } else if (appendageStick2.GetRawButtonPressed(7)) {
        SetGoal(kTopHatch);
    } else if (appendageStick2.GetRawButtonPressed(8)) {
        SetGoal(kTopCargo);
    } else if (appendageStick2.GetRawButtonPressed(2)) {
        SetGoal(kCargoShip);
    }
}

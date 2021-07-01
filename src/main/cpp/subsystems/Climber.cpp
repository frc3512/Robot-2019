// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <chrono>

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
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
}

void Climber::SetDriveVoltage(units::volt_t voltage) {
    m_drive.SetVoltage(voltage);
}

void Climber::SetVoltage(units::volt_t voltage) { m_lift.SetVoltage(voltage); }

void Climber::ResetEncoder() { m_encoder.Reset(); }

units::meter_t Climber::GetHeight() {
    return units::meter_t{m_encoder.GetDistance()};
}

units::meters_per_second_t Climber::GetVelocity() {
    return units::meters_per_second_t{m_encoder.GetRate()};
}

void Climber::ControllerPeriodic() {
    UpdateDt();

    m_observer.Predict(m_u, GetDt());
    Eigen::Matrix<double, 1, 1> y;
    y << GetHeight().to<double>();
    m_observer.Correct(m_controller.GetInputs(), y);

    m_u = m_controller.Calculate(m_observer.Xhat());

    SetVoltage(units::volt_t{m_u(0)});

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_climberSim.SetInput(frc::MakeMatrix<1, 1>(
            m_lift.Get() * frc::RobotController::GetInputVoltage()));

        m_climberSim.Update(GetDt());

        m_encoderSim.SetDistance(m_climberSim.GetOutput()(0));
    }
}

void Climber::AutonomousInit() {
    Enable();
    SetGoal(0.02_m);
}

void Climber::TeleopInit() {
    Enable();
    SetGoal(0.02_m);
}

void Climber::SetGoal(units::meter_t position) {
    m_controller.SetGoal(position);
}

bool Climber::AtGoal() const { return m_controller.AtGoal(); }

void Climber::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/FourBarLift.hpp"

#include <chrono>
#include <limits>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>

using namespace frc3512;
using namespace frc3512::Constants::FourBarLift;
using namespace std::chrono_literals;

FourBarLift::FourBarLift()
    : ControlledSubsystemBase{"FourBarLift",
                              {ControllerLabel{"Angle", "rad"},
                               ControllerLabel{"Angular Velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angle", "rad"}}} {
    m_grbx.Set(0.0);
    m_encoder.SetDistancePerPulse(kDpP);
    m_grbx.SetInverted(true);
    Reset();
    SetGoal(0.0_rad);
}

void FourBarLift::SetVoltage(units::volt_t voltage) {
    m_grbx.SetVoltage(voltage);
}

void FourBarLift::SetClimbing(bool on) { m_controller.SetClimbing(on); }

units::inch_t FourBarLift::GetHeight() {
    return units::inch_t{m_encoder.GetDistance()};
}

void FourBarLift::SetGoal(units::radian_t position) {
    m_controller.SetGoal(position);
}

bool FourBarLift::AtGoal() { return m_controller.AtGoal(); }

void FourBarLift::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_encoder.Reset();
}

void FourBarLift::ControllerPeriodic() {
    UpdateDt();

    m_observer.Predict(m_u, GetDt());
    Eigen::Matrix<double, 1, 1> y;
    y << GetHeight().to<double>();
    m_observer.Correct(m_u, y);

    m_u = m_controller.Calculate(m_observer.Xhat());

    SetVoltage(units::volt_t{m_u(0)});

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_fourBarLiftSim.SetInput(frc::MakeMatrix<1, 1>(
            m_grbx.Get() * frc::RobotController::GetInputVoltage()));
        m_fourBarLiftSim.Update(GetDt());
        m_encoderSim.SetDistance(m_fourBarLiftSim.GetAngle().to<double>());
    }
}

void FourBarLift::TeleopPeriodic() {
    static frc::Joystick driveStick2{Constants::Robot::kDriveStick2Port};
    static frc::Joystick appendageStick1{
        Constants::Robot::kAppendageStick1Port};
    static frc::Joystick appendageStick2{
        Constants::Robot::kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(3)) {
        SetGoal(units::radian_t{kMin});
    } else if (appendageStick2.GetRawButtonPressed(2)) {
        SetGoal(units::radian_t{kMax});
    } else if (appendageStick1.GetRawButtonPressed(11)) {
        SetGoal(units::radian_t{kBottomHatch});
    } else if (driveStick2.GetRawButtonPressed(7)) {
        m_controller.SetClimbing(true);
    } else if (driveStick2.GetRawButtonPressed(8)) {
        m_controller.SetClimbing(false);
    } else if (appendageStick1.GetRawButtonPressed(7) ||
               appendageStick1.GetRawButtonPressed(8) ||
               appendageStick1.GetRawButtonPressed(9) ||
               appendageStick1.GetRawButtonPressed(10) ||
               appendageStick1.GetRawButtonPressed(12)) {
        SetGoal(units::radian_t{kMax});
    }
}

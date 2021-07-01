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
    SetGoal(0.0);
}

void FourBarLift::SetVoltage(double voltage) { m_grbx.Set(voltage); }

void FourBarLift::SetClimbing(bool on) { m_controller.SetClimbing(on); }

void FourBarLift::ResetEncoder() { m_encoder.Reset(); }

double FourBarLift::GetHeight() { return m_encoder.GetDistance(); }

void FourBarLift::SetGoal(double position) { m_controller.SetGoal(position); }

bool FourBarLift::AtReference() const { return m_controller.AtReferences(); }

bool FourBarLift::AtGoal() { return m_controller.AtGoal(); }

void FourBarLift::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

void FourBarLift::ControllerPeriodic() {
    m_controller.SetMeasuredAngle(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_grbx.Set(m_controller.ControllerVoltage() / batteryVoltage);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_fourBarLiftSim.SetInput(frc::MakeMatrix<1, 1>(
            m_grbx.Get() * frc::RobotController::GetInputVoltage()));
        m_fourBarLiftSim.Update(GetDt());
        m_encoderSim.SetDistance(m_fourBarLiftSim.GetAngle().to<double>());
    }
}

void FourBarLift::TeleopPeriodic() {
    static frc::Joystick driveStick2{Constants::Robot::kDriveStick2Port};
    static frc::Joystick appendageStick1{Constants::Robot::kAppendageStickPort};
    static frc::Joystick appendageStick2{
        Constants::Robot::kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(3)) {
        SetGoal(kMin);
    } else if (appendageStick2.GetRawButtonPressed(2)) {
        SetGoal(kMax);
    } else if (appendageStick1.GetRawButtonPressed(11)) {
        SetGoal(kBottomHatch);
    } else if (driveStick2.GetRawButtonPressed(7)) {
        m_controller.SetClimbing(true);
    } else if (driveStick2.GetRawButtonPressed(8)) {
        m_controller.SetClimbing(false);
    } else if (appendageStick1.GetRawButtonPressed(7) ||
               appendageStick1.GetRawButtonPressed(8) ||
               appendageStick1.GetRawButtonPressed(9) ||
               appendageStick1.GetRawButtonPressed(10) ||
               appendageStick1.GetRawButtonPressed(12)) {
        SetGoal(kMax);
    }
}

// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <frc/Joystick.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "CurvatureDrive.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Drivetrain;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain()
    : ControlledSubsystemBase(
          "Drivetrain",
          {ControllerLabel{"X", "m"}, ControllerLabel{"Y", "m"},
           ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left velocity", "m/s"},
           ControllerLabel{"Right velocity", "m/s"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"},
           ControllerLabel{"Left voltage error", "V"},
           ControllerLabel{"Right voltage error", "V"},
           ControllerLabel{"Angular velocity error", "rad/s"}},
          {ControllerLabel{"Left voltage", "V"},
           ControllerLabel{"Right voltage", "V"}},
          {ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"}}) {
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);
    m_rightGrbx.SetInverted(false);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_rightEncoder.SetReverseDirection(true);

    m_leftEncoder.SetDistancePerPulse(kDpP);
    m_rightEncoder.SetDistancePerPulse(kDpP);

    ShiftUp();
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

void Drivetrain::ShiftUp() { m_shifter.Set(true); }

void Drivetrain::ShiftDown() { m_shifter.Set(false); }

void Drivetrain::Shift() { m_shifter.Set(!m_shifter.Get()); }

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{-m_gyro.GetAngle()};
}

units::radians_per_second_t Drivetrain::GetAngularRate() const {
    return units::degrees_per_second_t{m_gyro.GetRate()};
}

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

units::meter_t Drivetrain::GetLeftDisplacement() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightDisplacement() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftRate() const {
    return units::meters_per_second_t{m_leftEncoder.GetRate()};
}

units::meters_per_second_t Drivetrain::GetRightRate() const {
    return units::meters_per_second_t{m_rightEncoder.GetRate()};
}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
    m_leftGrbx.SetSafetyEnabled(true);
    m_rightGrbx.SetSafetyEnabled(true);
}

void Drivetrain::DisableController() {
    m_controller.Disable();
    m_leftGrbx.SetSafetyEnabled(false);
    m_rightGrbx.SetSafetyEnabled(false);
}

bool Drivetrain::IsControllerEnabled() const {
    return m_controller.IsEnabled();
}

void Drivetrain::Reset() {
    m_controller.Reset();
    ResetEncoders();
    ResetGyro();
}

void Drivetrain::SetWaypoints(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.SetWaypoints(waypoints);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

void Drivetrain::AutonomousInit() {
    Reset();
    EnableController();
    m_startTime = std::chrono::steady_clock::now();
}

void Drivetrain::ControllerPeriodic() {
    m_controller.SetMeasuredLocalOutputs(GetAngle(), GetLeftDisplacement(),
                                         GetRightDisplacement());
    auto now = std::chrono::steady_clock::now();
    m_controller.Update(now - m_lastTime, now - m_startTime);

    // Set motor inputs
    auto u = m_controller.GetInputs();
    SetLeftManual(u(0, 0) / 12.0);
    SetRightManual(u(1, 0) / 12.0);
    m_lastTime = now;
}

void Drivetrain::TeleopPeriodic() {
    static frc::Joystick driveStick1{kDriveStick1Port};
    static frc::Joystick driveStick2{kDriveStick2Port};

    double y = ApplyDeadband(-driveStick1.GetY(), kJoystickDeadband);
    double x = ApplyDeadband(driveStick2.GetX(), kJoystickDeadband);

    if (driveStick2.GetRawButtonPressed(1)) {
        Shift();
    }

    auto [left, right] = CurvatureDrive(y, x, driveStick2.GetRawButton(2));

    Eigen::Matrix<double, 2, 1> u =
        frc::MakeMatrix<2, 1>(left * 12.0, right * 12.0);

    m_leftGrbx.SetVoltage(units::volt_t{u(0)});
    m_rightGrbx.SetVoltage(units::volt_t{u(1)});
}

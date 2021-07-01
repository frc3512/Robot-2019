// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
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

    frc::SmartDashboard::PutData(&m_field);
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

void Drivetrain::Reset() {
    m_controller.Reset();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_gyro.Reset();

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_drivetrainSim.SetState(Eigen::Matrix<double, 7, 1>::Zero());
        m_field.SetRobotPose(frc::Pose2d(0_m, 0_m, 0_rad));
    }
}

void Drivetrain::SetWaypoints(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.SetWaypoints(waypoints);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

void Drivetrain::AutonomousInit() {
    Reset();
    Enable();
    m_startTime = std::chrono::steady_clock::now();
}

void Drivetrain::ControllerPeriodic() {
    UpdateDt();

    m_controller.SetMeasuredLocalOutputs(GetAngle(), GetLeftDisplacement(),
                                         GetRightDisplacement());
    auto now = std::chrono::steady_clock::now();
    m_controller.Update(GetDt(), now - m_startTime);

    // Set motor inputs
    auto u = m_controller.GetInputs();
    SetLeftManual(u(0, 0) / 12.0);
    SetRightManual(u(1, 0) / 12.0);

    // Log(m_controller.GetReferences(), m_observer.Xhat(), u,
    //     m_controller.GetOutputs());

    if constexpr (frc::RobotBase::IsSimulation()) {
        auto batteryVoltage = frc::RobotController::GetInputVoltage();
        m_drivetrainSim.SetInputs(
            units::volt_t{std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage},
            units::volt_t{std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage});

        m_drivetrainSim.Update(GetDt());

        m_leftEncoderSim.SetDistance(
            m_drivetrainSim.GetLeftPosition().to<double>());
        m_rightEncoderSim.SetDistance(
            m_drivetrainSim.GetRightPosition().to<double>());
        m_gyroSim.SetAngle(
            units::degree_t{m_drivetrainSim.GetHeading().Radians()});

        m_gyroSim.SetAngle(
            -units::degree_t{m_drivetrainSim.GetHeading().Radians()});

        m_field.SetRobotPose(m_drivetrainSim.GetPose());
    }
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

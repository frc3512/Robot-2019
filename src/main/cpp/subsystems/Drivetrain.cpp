// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
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

void Drivetrain::ShiftUp() { m_shifter.Set(true); }

void Drivetrain::ShiftDown() { m_shifter.Set(false); }

void Drivetrain::Shift() { m_shifter.Set(!m_shifter.Get()); }

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{-m_gyro.GetAngle()};
}

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

units::meter_t Drivetrain::GetLeftPosition() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightPosition() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftRate() const {
    return units::meters_per_second_t{m_leftEncoder.GetRate()};
}

units::meters_per_second_t Drivetrain::GetRightRate() const {
    return units::meters_per_second_t{m_rightEncoder.GetRate()};
}

void Drivetrain::Reset(frc::Pose2d initialPose) {
    m_observer.Reset();
    m_odometer.ResetPosition(initialPose, GetAngle());
    m_controller.Reset(initialPose);
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_gyro.Reset();

    Eigen::Matrix<double, 10, 1> xHat;
    xHat(0, 0) = initialPose.Translation().X().to<double>();
    xHat(1, 0) = initialPose.Translation().Y().to<double>();
    xHat(2, 0) = initialPose.Rotation().Radians().to<double>();
    xHat.block<7, 1>(3, 0).setZero();
    m_observer.SetXhat(xHat);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_drivetrainSim.SetState(Eigen::Matrix<double, 7, 1>::Zero());
        m_field.SetRobotPose(frc::Pose2d(0_m, 0_m, 0_rad));
    }
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end) {
    m_controller.AddTrajectory(start, interior, end);
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(start, interior, end, config);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.AddTrajectory(waypoints);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(waypoints, config);
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig() {
    return DrivetrainController::MakeTrajectoryConfig();
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig(
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity) {
    return DrivetrainController::MakeTrajectoryConfig(startVelocity,
                                                      endVelocity);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

void Drivetrain::TeleopInit() {
    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    Enable();
}

void Drivetrain::ControllerPeriodic() {
    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    Eigen::Matrix<double, 3, 1> y;
    y << frc::AngleModulus(GetAngle()).to<double>(),
        GetLeftPosition().to<double>(), GetRightPosition().to<double>();
    m_observer.Correct(m_controller.GetInputs(), y);
    m_odometer.Update(
        units::radian_t{y(DrivetrainController::LocalOutput::kHeading)},
        units::meter_t{y(DrivetrainController::LocalOutput::kLeftPosition)},
        units::meter_t{y(DrivetrainController::LocalOutput::kRightPosition)});

    if (m_controller.HaveTrajectory()) {
        m_u = m_controller.Calculate(m_observer.Xhat());

        if (!AtGoal()) {
            m_leftGrbx.SetVoltage(units::volt_t{m_u(0)});
            m_rightGrbx.SetVoltage(units::volt_t{m_u(1)});
        } else {
            m_leftGrbx.SetVoltage(0_V);
            m_rightGrbx.SetVoltage(0_V);
        }
    } else {
        // Update previous u stored in the controller. We don't care what the
        // return value is.
        static_cast<void>(m_controller.Calculate(m_observer.Xhat()));

        // Run observer predict with inputs from teleop
        m_u << std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                   frc::RobotController::GetInputVoltage(),
            std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                frc::RobotController::GetInputVoltage();
    }

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

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

void Drivetrain::TestPeriodic() {
    static frc::Joystick driveStick1{kDriveStick1Port};
    static frc::Joystick driveStick2{kDriveStick2Port};

    double y = ApplyDeadband(-driveStick1.GetY(), kJoystickDeadband);
    double x = ApplyDeadband(driveStick2.GetX(), kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    auto [left, right] = CurvatureDrive(y, x, driveStick2.GetRawButton(2));

    Eigen::Matrix<double, 2, 1> u =
        frc::MakeMatrix<2, 1>(left * 12.0, right * 12.0);

    m_leftGrbx.SetVoltage(units::volt_t{u(0)});
    m_rightGrbx.SetVoltage(units::volt_t{u(1)});
}

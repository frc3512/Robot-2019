// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/QR>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/NumericalJacobian.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h>
#include <wpi/MathExtras.h>

using namespace frc3512;
using namespace frc3512::Constants;
using namespace frc3512::Constants::Drivetrain;

frc::LinearSystem<2, 2, 2> DrivetrainController::m_plant{GetPlant()};

DrivetrainController::DrivetrainController(const std::array<double, 5>& Qelems,
                                           const std::array<double, 2>& Relems,
                                           units::second_t dt) {
    Eigen::Matrix<double, 10, 1> x0;
    x0.setZero();
    x0(State::kLeftVelocity, 0) = 1e-9;
    x0(State::kRightVelocity, 0) = 1e-9;
    Eigen::Matrix<double, 10, 1> x1;
    x1.setZero();
    x1(State::kLeftVelocity, 0) = 1;
    x1(State::kRightVelocity, 0) = 1;
    Eigen::Matrix<double, 2, 1> u0;
    u0.setZero();

    Eigen::Matrix<double, 5, 5> A0 =
        frc::NumericalJacobianX<10, 10, 2>(Dynamics, x0, u0).block<5, 5>(0, 0);
    Eigen::Matrix<double, 5, 5> A1 =
        frc::NumericalJacobianX<10, 10, 2>(Dynamics, x1, u0).block<5, 5>(0, 0);
    m_B =
        frc::NumericalJacobianU<10, 10, 2>(Dynamics, x0, u0).block<5, 2>(0, 0);

    m_K0 = frc::LinearQuadraticRegulator<5, 2>(A0, m_B, Qelems, Relems, dt).K();
    m_K1 = frc::LinearQuadraticRegulator<5, 2>(A1, m_B, Qelems, Relems, dt).K();

    m_trajectoryTimeElapsed.Start();
}

void DrivetrainController::AddTrajectory(
    const frc::Pose2d& start, const std::vector<frc::Translation2d>& interior,
    const frc::Pose2d& end, const frc::TrajectoryConfig& config) {
    bool hadTrajectory = HaveTrajectory();

    m_trajectory = m_trajectory + frc::TrajectoryGenerator::GenerateTrajectory(
                                      start, interior, end, config);
    m_goal = m_trajectory.States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (!hadTrajectory) {
        m_trajectoryTimeElapsed.Reset();
    }
}

void DrivetrainController::AddTrajectory(
    const std::vector<frc::Pose2d>& waypoints,
    const frc::TrajectoryConfig& config) {
    bool hadTrajectory = HaveTrajectory();

    m_trajectory = m_trajectory + frc::TrajectoryGenerator::GenerateTrajectory(
                                      waypoints, config);
    m_goal = m_trajectory.States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (!hadTrajectory) {
        m_trajectoryTimeElapsed.Reset();
    }
}

bool DrivetrainController::HaveTrajectory() const {
    return m_trajectory.States().size() > 0;
}

void DrivetrainController::AbortTrajectories() {
    m_trajectory = frc::Trajectory{};
}

bool DrivetrainController::AtGoal() const {
    frc::Pose2d ref{units::meter_t{m_r(State::kX, 0)},
                    units::meter_t{m_r(State::kY, 0)},
                    units::radian_t{m_r(State::kHeading, 0)}};
    return m_goal == ref &&
           m_trajectoryTimeElapsed.Get() >= m_trajectory.TotalTime() &&
           m_atReferences;
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    Eigen::Matrix<double, 10, 1> xHat;
    xHat(0) = initialPose.X().to<double>();
    xHat(1) = initialPose.Y().to<double>();
    xHat(2) = initialPose.Rotation().Radians().to<double>();
    xHat.block<4, 1>(6, 0).setZero();

    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;
    m_goal = initialPose;

    UpdateAtReferences(Eigen::Matrix<double, 5, 1>::Zero());
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Calculate(
    const Eigen::Matrix<double, 10, 1>& x) {
    m_u << 0.0, 0.0;

    if (HaveTrajectory()) {
        frc::Trajectory::State ref =
            m_trajectory.Sample(m_trajectoryTimeElapsed.Get());

        auto [vlRef, vrRef] =
            ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

        m_nextR << ref.pose.X().to<double>(), ref.pose.Y().to<double>(),
            ref.pose.Rotation().Radians().to<double>(), vlRef.to<double>(),
            vrRef.to<double>(), 0, 0, 0, 0, 0;

        // Compute feedforward
        /* Eigen::Matrix<double, 5, 1> rdot = (m_nextR - m_r) /
        dt.to<double>(); Eigen::Matrix<double, 10, 1> rAugmented;
        rAugmented.block<5, 1>(0, 0) = m_r;
        rAugmented.block<5, 1>(5, 0).setZero();
        Eigen::Matrix<double, 2, 1> uff = m_B.householderQr().solve(
            rdot - Dynamics(rAugmented, Eigen::Matrix<double, 2,
        1>::Zero()) .block<5, 1>(0, 0));
        */

        // Use built in feedforward and hope it works
        Eigen::Matrix<double, 2, 1> u_fb = Controller(x, m_nextR);
        u_fb = frc::NormalizeInputVector<2>(u_fb, 12.0);
        m_u = u_fb + m_ff.Calculate(m_nextR);
        m_u = frc::NormalizeInputVector<2>(m_u, 12.0);

        Eigen::Matrix<double, 5, 1> error =
            m_r.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
        error(State::kHeading) =
            frc::AngleModulus(units::radian_t{error(State::kHeading)})
                .to<double>();
        UpdateAtReferences(error);

        m_r = m_nextR;
    }

    if (AtGoal() && HaveTrajectory()) {
        m_trajectory = frc::Trajectory{};
        m_trajectoryTimeElapsed.Reset();
    }

    return m_u;
}

frc::LinearSystem<2, 2, 2> DrivetrainController::GetPlant() {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(
        Constants::Drivetrain::kLinearV, Constants::Drivetrain::kLinearA,
        Constants::Drivetrain::kAngularV, Constants::Drivetrain::kAngularA);
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig() {
    return MakeTrajectoryConfig(0_mps, 0_mps);
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig(
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity) {
    frc::TrajectoryConfig config{kMaxV, kMaxA - 14.5_mps_sq};

    config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint{
        m_plant, frc::DifferentialDriveKinematics{kWidth}, 8_V});

    // Slows drivetrain down on curves to avoid understeer that
    // introduces odometry errors
    config.AddConstraint(frc::CentripetalAccelerationConstraint{3_mps_sq});

    config.SetStartVelocity(startVelocity);
    config.SetEndVelocity(endVelocity);

    return config;
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 10, 1>& r) {
    double kx = m_K0(0, 0);
    double ky0 = m_K0(0, 1);
    double kvpos0 = m_K0(0, 3);
    double kvneg0 = m_K0(1, 3);
    double ky1 = m_K1(0, 1);
    double ktheta1 = m_K1(0, 2);
    double kvpos1 = m_K1(0, 3);

    double v = (x(State::kLeftVelocity, 0) + x(State::kRightVelocity, 0)) / 2.0;
    double sqrtAbsV = std::sqrt(std::abs(v));

    Eigen::Matrix<double, 2, 5> K;
    K(0, 0) = kx;
    K(0, 1) = (ky0 + (ky1 - ky0) * sqrtAbsV) * wpi::sgn(v);
    K(0, 2) = ktheta1 * sqrtAbsV;
    K(0, 3) = kvpos0 + (kvpos1 - kvpos0) * sqrtAbsV;
    K(0, 4) = kvneg0 - (kvpos1 - kvpos0) * sqrtAbsV;
    K(1, 0) = kx;
    K(1, 1) = -K(0, 1);
    K(1, 2) = -K(0, 2);
    K(1, 3) = K(0, 4);
    K(1, 4) = K(0, 3);

    Eigen::Matrix<double, 2, 1> uError;
    uError << x(State::kLeftVoltageError, 0), x(State::kRightVoltageError, 0);

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(2, 0));
    inRobotFrame(0, 1) = std::sin(x(2, 0));
    inRobotFrame(1, 0) = -std::sin(x(2, 0));
    inRobotFrame(1, 1) = std::cos(x(2, 0));

    Eigen::Matrix<double, 5, 1> error =
        r.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
    error(State::kHeading, 0) = NormalizeAngle(error(State::kHeading, 0));
    return K * inRobotFrame * error;
}

Eigen::Matrix<double, 10, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    // constexpr auto motors = frc::DCMotor::MiniCIM(3);

    // constexpr units::dimensionless_t Glow = 15.32;  // Low gear ratio
    // constexpr units::dimensionless_t Ghigh = 7.08;  // High gear
    // ratio constexpr auto r = 0.0746125_m;                 // Wheel
    // radius constexpr auto m = 63.503_kg;                   // Robot
    // mass constexpr auto J = 5.6_kg_sq_m;                 // Robot
    // moment of inertia

    // constexpr auto C1 =
    //     -1.0 * Ghigh * Ghigh * motors.Kt / (motors.Kv * motors.R * r
    //     * r);
    // constexpr auto C2 = Ghigh * motors.Kt / (motors.R * r);
    // constexpr auto k1 = (1 / m + rb * rb / J);
    // constexpr auto k2 = (1 / m - rb * rb / J);

    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = m_plant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 7> A;
    A.block<2, 2>(0, 0) = m_plant.A();

    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();
    A.block<4, 2>(0, 4) = B;
    A.block<4, 1>(0, 6) << 0, 0, 1, -1;

    double v = (x(State::kLeftVelocity, 0) + x(State::kRightVelocity, 0)) / 2.0;

    Eigen::Matrix<double, 10, 1> result;
    result(0, 0) = v * std::cos(x(State::kHeading, 0));
    result(1, 0) = v * std::sin(x(State::kHeading, 0));
    result(2, 0) = ((x(State::kRightVelocity, 0) - x(State::kLeftVelocity, 0)) /
                    (2.0 * rb))
                       .to<double>();
    result.block<4, 1>(3, 0) = A * x.block<7, 1>(3, 0) + B * u;
    result.block<3, 1>(7, 0).setZero();
    return result;
}

Eigen::Matrix<double, 3, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 3, 1> y;
    y << x(State::kHeading, 0), x(State::kLeftPosition, 0),
        x(State::kRightPosition, 0);
    return y;
}

Eigen::Matrix<double, 6, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 6, 1> y;
    y.block<3, 1>(0, 0) = x.block<3, 1>(0, 0);
    y(3, 0) = x(State::kLeftPosition, 0);
    y(4, 0) = x(State::kRightPosition, 0);
    y(5, 0) = (x(State::kRightVelocity, 0) - x(State::kLeftVelocity, 0)) /
              (2.0 * rb.to<double>());
    return y;
}

void DrivetrainController::UpdateAtReferences(
    const Eigen::Matrix<double, 5, 1>& error) {
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kPositionTolerance &&
                     std::abs(error(2, 0)) < kAngleTolerance &&
                     std::abs(error(3, 0)) < kVelocityTolerance &&
                     std::abs(error(4, 0)) < kVelocityTolerance;
}

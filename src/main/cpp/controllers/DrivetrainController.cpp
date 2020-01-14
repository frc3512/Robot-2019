// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/QR>
#include <frc/RobotController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/NumericalJacobian.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <units/units.h>
#include <wpi/MathExtras.h>

using namespace frc3512;
using namespace frc3512::Constants;
using namespace frc3512::Constants::Drivetrain;

DrivetrainController::DrivetrainController(const std::array<double, 5>& Qelems,
                                           const std::array<double, 2>& Relems,
                                           units::second_t dt) {
    m_y.setZero();
    Reset();

    Eigen::Matrix<double, 5, 1> x0;
    x0 << 0, 0, 0, 1e-9, 1e-9;
    Eigen::Matrix<double, 5, 1> x1;
    x1 << 0, 0, 0, 1, 1;
    Eigen::Matrix<double, 2, 1> u0;
    u0.setZero();

    auto A0 = frc::NumericalJacobianX<5, 5, 2>(Dynamics, x0, u0);
    auto A1 = frc::NumericalJacobianX<5, 5, 2>(Dynamics, x1, u0);
    m_B = frc::NumericalJacobianU<5, 5, 2>(Dynamics, x0, u0);

    m_K0 = frc::LinearQuadraticRegulator<5, 2>(A0, m_B, Qelems, Relems, dt).K();
    m_K1 = frc::LinearQuadraticRegulator<5, 2>(A1, m_B, Qelems, Relems, dt).K();
}

void DrivetrainController::Enable() { m_isEnabled = true; }

void DrivetrainController::Disable() { m_isEnabled = false; }

bool DrivetrainController::IsEnabled() const { return m_isEnabled; }

void DrivetrainController::SetWaypoints(
    const std::vector<frc::Pose2d>& waypoints) {
    std::lock_guard lock(m_trajectoryMutex);
    m_goal = waypoints.back();
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        waypoints, frc::TrajectoryConfig{kMaxV, kMaxA});
}

bool DrivetrainController::AtGoal() {
    frc::Pose2d ref{units::meter_t{m_r(0, 0)}, units::meter_t{m_r(1, 0)},
                    units::radian_t{m_r(2, 0)}};
    return m_goal == ref && m_atReferences;
}

void DrivetrainController::SetMeasuredLocalOutputs(
    units::radian_t heading, units::meters_per_second_t leftVelocity,
    units::meters_per_second_t rightVelocity) {
    m_y << heading.to<double>(), leftVelocity.to<double>(),
        rightVelocity.to<double>();
}

void DrivetrainController::SetMeasuredGlobalOutputs(
    units::meter_t x, units::meter_t y, units::radian_t heading,
    units::meters_per_second_t leftVelocity,
    units::meters_per_second_t rightVelocity) {
    m_y << x.to<double>(), y.to<double>(), heading.to<double>(),
        leftVelocity.to<double>(), rightVelocity.to<double>();
}

Eigen::Matrix<double, 3, 1> DrivetrainController::EstimatedLocalOutputs()
    const {
    return LocalMeasurementModel(m_observer.Xhat(),
                                 Eigen::Matrix<double, 2, 1>::Zero());
}

Eigen::Matrix<double, 5, 1> DrivetrainController::EstimatedGlobalOutputs()
    const {
    return GlobalMeasurementModel(m_observer.Xhat(),
                                  Eigen::Matrix<double, 2, 1>::Zero());
}

units::volt_t DrivetrainController::ControllerLeftVoltage() const {
    if (m_isEnabled) {
        return units::volt_t{m_cappedU(0, 0)};
    } else {
        return 0_V;
    }
}

units::volt_t DrivetrainController::ControllerRightVoltage() const {
    if (m_isEnabled) {
        return units::volt_t{m_cappedU(1, 0)};
    } else {
        return 0_V;
    }
}

units::meters_per_second_t DrivetrainController::EstimatedLeftVelocity() const {
    return units::meters_per_second_t{m_observer.Xhat(3)};
}

units::meters_per_second_t DrivetrainController::EstimatedRightVelocity()
    const {
    return units::meters_per_second_t{m_observer.Xhat(4)};
}

units::meters_per_second_t DrivetrainController::LeftVelocityError() const {
    return units::meters_per_second_t{m_r(3, 0) - m_observer.Xhat(3)};
}

units::meters_per_second_t DrivetrainController::RightVelocityError() const {
    return units::meters_per_second_t{m_r(4, 0) - m_observer.Xhat(4)};
}

frc::Pose2d DrivetrainController::EstimatedPose() const {
    // TODO: use local or global depending on measurements available
    const auto& xHat = m_observer.Xhat();
    return frc::Pose2d{units::meter_t{xHat(0, 0)}, units::meter_t{xHat(1, 0)},
                       units::radian_t{xHat(2, 0)}};
}

units::meters_per_second_t DrivetrainController::LeftVelocityReference() const {
    return units::meters_per_second_t{m_nextR(3, 0)};
}

units::meters_per_second_t DrivetrainController::RightVelocityReference()
    const {
    return units::meters_per_second_t{m_nextR(4, 0)};
}

void DrivetrainController::Update(units::second_t dt,
                                  units::second_t elaspedTime) {
    frc::Trajectory::State ref;
    {
        std::lock_guard lock(m_trajectoryMutex);
        ref = m_trajectory.Sample(elaspedTime);
    }

    voltageLogger.Log(elaspedTime, ControllerLeftVoltage().to<double>(),
                      ControllerRightVoltage().to<double>(),
                      frc::RobotController::GetInputVoltage());
    velocityLogger.Log(elaspedTime, m_y(1, 0), m_y(2, 0),
                       EstimatedLeftVelocity().to<double>(),
                       EstimatedRightVelocity().to<double>(),
                       LeftVelocityReference().to<double>(),
                       RightVelocityReference().to<double>());
    positionLogger.Log(elaspedTime,
                       EstimatedPose().Translation().X().to<double>(),
                       EstimatedPose().Translation().Y().to<double>(),
                       EstimatedPose().Rotation().Radians().to<double>(),
                       ref.pose.Translation().X().to<double>(),
                       ref.pose.Translation().Y().to<double>(),
                       ref.pose.Rotation().Radians().to<double>());
    errorCovLogger.Log(elaspedTime, m_observer.P(0, 0), m_observer.P(1, 1),
                       m_observer.P(2, 2), m_observer.P(3, 3),
                       m_observer.P(4, 4));

    // clang-format off
    // v = (v_r + v_l) / 2     (1)
    // w = (v_r - v_l) / (2r)  (2)
    // k = w / v               (3)
    //
    // v_l = v - wr
    // v_l = v - (vk)r
    // v_l = v(1 - kr)
    //
    // v_r = v + wr
    // v_r = v + (vk)r
    // v_r = v(1 + kr)
    // clang-format on
    constexpr auto r = kWidth / 2.0;
    units::meters_per_second_t vl =
        ref.velocity * (1 - (ref.curvature * r).to<double>());
    units::meters_per_second_t vr =
        ref.velocity * (1 + (ref.curvature * r).to<double>());
    m_nextR(0, 0) = ref.pose.Translation().X().to<double>();
    m_nextR(1, 0) = ref.pose.Translation().Y().to<double>();
    m_nextR(2, 0) = ref.pose.Rotation().Radians().to<double>();
    m_nextR(3, 0) = vl.to<double>();
    m_nextR(4, 0) = vr.to<double>();

    // Compute feedforward
    Eigen::Matrix<double, 5, 1> rdot = (m_nextR - m_r) / dt.to<double>();
    Eigen::Matrix<double, 2, 1> uff = m_B.householderQr().solve(
        rdot - Dynamics(m_r, Eigen::Matrix<double, 2, 1>::Zero()));

    Eigen::Matrix<double, 2, 1> u =
        Controller(m_observer.Xhat(), m_nextR) + uff;

    double Vl = u(0, 0);
    double Vr = u(1, 0);
    double max = std::max(std::abs(Vr), std::abs(Vl));
    if (max > 12.0) {
        Vr = Vr * 12.0 / max;
        Vl = Vl * 12.0 / max;
    }

    m_cappedU << Vl, Vr;

    m_observer.Correct(m_cappedU, m_y);

    auto error = m_r - m_observer.Xhat();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kPositionTolerance &&
                     std::abs(error(2, 0)) < kAngleTolerance &&
                     std::abs(error(3, 0)) < kVelocityTolerance &&
                     std::abs(error(4, 0)) < kVelocityTolerance;

    m_r = m_nextR;
    m_observer.Predict(m_cappedU, dt);
}

void DrivetrainController::Reset() {
    m_observer.Reset();
    m_r.setZero();
    m_nextR.setZero();
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    m_observer.Reset();

    Eigen::Matrix<double, 5, 1> xHat;
    xHat(0, 0) = initialPose.Translation().X().to<double>();
    xHat(1, 0) = initialPose.Translation().Y().to<double>();
    xHat(2, 0) = initialPose.Rotation().Radians().to<double>();
    xHat(3, 0) = 0.0;
    xHat(4, 0) = 0.0;
    m_observer.SetXhat(xHat);

    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 5, 1>& r) {
    double kx = m_K0(0, 0);
    double ky0 = m_K0(0, 1);
    double kvpos0 = m_K0(0, 3);
    double kvneg0 = m_K0(1, 3);
    double ky1 = m_K1(0, 1);
    double ktheta1 = m_K1(0, 2);
    double kvpos1 = m_K1(0, 3);

    double v = (x(3, 0) + x(4, 0)) / 2.0;
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

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(2, 0));
    inRobotFrame(0, 1) = std::sin(x(2, 0));
    inRobotFrame(1, 0) = -std::sin(x(2, 0));
    inRobotFrame(1, 1) = std::cos(x(2, 0));
    return K * inRobotFrame * (r - x);
}

Eigen::Matrix<double, 5, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    constexpr auto motors = frc::DCMotor::MiniCIM(2);

    // constexpr units::dimensionless_t Glow = 15.32;  // Low gear ratio
    constexpr units::dimensionless_t Ghigh = 7.08;  // High gear ratio
    constexpr auto rb = 0.8382_m / 2.0;             // Robot radius
    constexpr auto r = 0.0746125_m;                 // Wheel radius
    constexpr auto m = 63.503_kg;                   // Robot mass
    constexpr auto J = 5.6_kg_sq_m;                 // Robot moment of inertia

    constexpr auto C1 =
        -1.0 * Ghigh * Ghigh * motors.Kt / (motors.Kv * motors.R * r * r);
    constexpr auto C2 = Ghigh * motors.Kt / (motors.R * r);
    constexpr auto k1 = (1 / m + rb * rb / J);
    constexpr auto k2 = (1 / m - rb * rb / J);

    units::meters_per_second_t vl{x(3, 0)};
    units::meters_per_second_t vr{x(4, 0)};

    Eigen::Matrix<double, 2, 2> A;
    A(0, 0) = k1.to<double>() * C1.to<double>();
    A(0, 1) = k2.to<double>() * C1.to<double>();
    A(1, 0) = k2.to<double>() * C1.to<double>();
    A(1, 1) = k1.to<double>() * C1.to<double>();
    Eigen::Matrix<double, 2, 2> B;
    B << k1.to<double>() * C2.to<double>(), k2.to<double>() * C2.to<double>(),
        k2.to<double>() * C2.to<double>(), k1.to<double>() * C2.to<double>();

    Eigen::Matrix<double, 5, 1> result;
    auto v = 0.5 * (vl + vr);
    result(0, 0) = v.to<double>() * std::cos(x(2, 0));
    result(1, 0) = v.to<double>() * std::sin(x(2, 0));
    result(2, 0) = ((vr - vl) / (2.0 * rb)).to<double>();
    result.block<2, 1>(3, 0) = A * x.block<2, 1>(3, 0) + B * u;
    return result;
}

Eigen::Matrix<double, 3, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);
    Eigen::Matrix<double, 3, 1> y;
    y << x(2, 0), x(3, 0), x(4, 0);
    return y;
}

Eigen::Matrix<double, 5, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);
    return x.block<5, 1>(0, 0);
}

// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <frc/DriverStation.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>

using namespace frc3512;
using namespace frc3512::Constants;
using namespace frc3512::Constants::Drivetrain;

DrivetrainController::DrivetrainController(const std::array<double, 5>& Qelems,
                                           const std::array<double, 2>& Relems,
                                           units::second_t dt) {
    constexpr int States = 5;
    constexpr int Inputs = 2;

    m_y.setZero();
    Reset();

    Eigen::Matrix<double, States, 1> x0;
    x0.setZero();
    Eigen::Matrix<double, States, 1> x1;
    x1 << 0, 0, 0, 1, 1;
    Eigen::Matrix<double, Inputs, 1> u0;
    u0.setZero();

    auto A0 = frc::NumericalJacobianX<States, States, Inputs>(Dynamics, x0, u0);
    auto A1 = frc::NumericalJacobianX<States, States, Inputs>(Dynamics, x1, u0);
    auto B = frc::NumericalJacobianU<States, States, Inputs>(Dynamics, x0, u0);

    // Matrices are blocked here to minimize matrix exponentiation calculations
    Eigen::Matrix<double, States + Inputs, States + Inputs> Mcont;
    Mcont.setZero();
    Mcont.template block<States, States>(0, 0) = A0 * dt.to<double>();
    Mcont.template block<States, Inputs>(0, States) = B * dt.to<double>();

    // Discretize A and B with the given timestep
    Eigen::Matrix<double, States + Inputs, States + Inputs> Mdisc = Mcont.exp();
    Eigen::Matrix<double, States, States> discA0 =
        Mdisc.template block<States, States>(0, 0);
    Eigen::Matrix<double, States, Inputs> discB =
        Mdisc.template block<States, Inputs>(0, States);

    Eigen::Matrix<double, States, States> discA1 = (A1 * dt.to<double>()).exp();

    m_K0 = frc::LinearQuadraticRegulator<5, 2, 3>(discA0, discB, Qelems, Relems,
                                                  kDt)
               .K();
    m_K1 = frc::LinearQuadraticRegulator<5, 2, 3>(discA1, discB, Qelems, Relems,
                                                  kDt)
               .K();
}

void DrivetrainController::Enable() { m_isEnabled = true; }

void DrivetrainController::Disable() { m_isEnabled = false; }

bool DrivetrainController::IsEnabled() const { return m_isEnabled; }

void DrivetrainController::SetWaypoints(
    const std::vector<frc::Pose2d>& waypoints) {
    std::lock_guard<wpi::mutex> lock(m_trajectoryMutex);
    m_goal = waypoints.back();
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        waypoints, frc::TrajectoryConfig{kMaxV, kMaxA});
    m_startTime = std::chrono::steady_clock::now();
}

bool DrivetrainController::AtGoal() {
    frc::Pose2d ref{units::meter_t{m_r(0, 0)}, units::meter_t{m_r(1, 0)},
                    units::radian_t{m_r(2, 0)}};
    return m_goal == ref && m_atReferences;
}

void DrivetrainController::SetMeasuredStates(double leftVelocity,
                                             double rightVelocity,
                                             double heading) {
    m_y << leftVelocity, rightVelocity, heading;
}

double DrivetrainController::ControllerLeftVoltage() const {
    if (m_isEnabled) {
        return m_cappedU(0, 0);
    } else {
        return 0.0;
    }
}

double DrivetrainController::ControllerRightVoltage() const {
    if (m_isEnabled) {
        return m_cappedU(1, 0);
    } else {
        return 0.0;
    }
}

double DrivetrainController::EstimatedLeftVelocity() const {
    // TODO: use local or global depending on measurements available
    auto& observer = m_localObserver;
    return observer.Xhat(3);
}

double DrivetrainController::EstimatedRightVelocity() const {
    // TODO: use local or global depending on measurements available
    auto& observer = m_localObserver;
    return observer.Xhat(4);
}

double DrivetrainController::LeftVelocityError() const {
    // TODO: use local or global depending on measurements available
    auto& observer = m_localObserver;
    return m_r(3, 0) - observer.Xhat(3);
}

double DrivetrainController::RightVelocityError() const {
    // TODO: use local or global depending on measurements available
    auto& observer = m_localObserver;
    return m_r(4, 0) - observer.Xhat(4);
}

frc::Pose2d DrivetrainController::EstimatedPose() const {
    // TODO: use local or global depending on measurements available
    const auto& observer = m_localObserver;
    const auto& xHat = observer.Xhat();
    return frc::Pose2d{units::meter_t{xHat(0, 0)}, units::meter_t{xHat(1, 0)},
                       units::radian_t{xHat(2, 0)}};
}

double DrivetrainController::LeftVelocityReference() { return m_nextR(3, 0); }

double DrivetrainController::RightVelocityReference() { return m_nextR(4, 0); }

void DrivetrainController::Update() {
    frc::Trajectory::State ref;
    {
        std::lock_guard<wpi::mutex> lock(m_trajectoryMutex);
        ref =
            m_trajectory.Sample(std::chrono::steady_clock::now() - m_startTime);
    }

    voltageLogger.Log(ControllerLeftVoltage(), ControllerRightVoltage(),
                      frc::DriverStation::GetInstance().GetBatteryVoltage());
    velocityLogger.Log(m_y(0, 0), m_y(1, 0), EstimatedLeftVelocity(),
                       EstimatedRightVelocity(), ref.velocity.to<double>(),
                       (ref.velocity * ref.curvature).to<double>(),
                       LeftVelocityReference(), RightVelocityReference());
    positionLogger.Log(m_leftPos, m_rightPos,
                       ref.pose.Translation().X().to<double>(),
                       ref.pose.Translation().Y().to<double>(),
                       ref.pose.Rotation().Radians().to<double>(),
                       m_goal.Translation().X().to<double>(),
                       m_goal.Translation().Y().to<double>(),
                       m_goal.Rotation().Radians().to<double>());

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

    // TODO: use local or global depending on measurements available
    auto& observer = m_localObserver;
    auto u = Controller(observer.Xhat(), m_nextR);

    double Vl = u(0, 0);
    double Vr = u(1, 0);
    double max = std::max(std::abs(Vr), std::abs(Vl));
    if (max > 12.0) {
        Vr = Vr * 12.0 / max;
        Vl = Vl * 12.0 / max;
    }

    const double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();

    m_filteredVoltage =
        kAlpha * m_filteredVoltage + (1 - kAlpha) * batteryVoltage;

    Vr = std::clamp(Vr / m_filteredVoltage, -1.0, 1.0);
    Vl = std::clamp(Vl / m_filteredVoltage, -1.0, 1.0);

    Vr = Vr * batteryVoltage;
    Vl = Vl * batteryVoltage;

    m_cappedU << Vl, Vr;

    observer.Correct(m_cappedU, m_y);

    auto error = m_r - observer.Xhat();
    m_atReferences = std::abs(error(0, 0)) < kVelocityTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_r = m_nextR;
    observer.Predict(m_cappedU, kDt);
}

void DrivetrainController::Reset() {
    m_localObserver.Reset();
    m_globalObserver.Reset();
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
    double sign = v >= 0 ? 1 : -1;

    Eigen::Matrix<double, 2, 5> K;
    K(0, 0) = kx;
    K(1, 0) = kx;
    K(0, 1) = (ky0 + (ky1 - ky0) * std::sqrt(v)) * sign;
    K(1, 1) = -K(0, 1);
    K(0, 2) = ktheta1 * std::sqrt(v);
    K(1, 2) = -K(0, 2);
    K(0, 3) = kvpos0 + (kvpos1 - kvpos0) * std::sqrt(v);
    K(1, 3) = kvneg0 - (kvpos1 - kvpos0) * std::sqrt(v);
    K(0, 4) = K(1, 3);
    K(1, 4) = K(0, 3);

    double heading = x(2, 0);
    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(heading);
    inRobotFrame(0, 1) = std::sin(heading);
    inRobotFrame(1, 0) = -std::sin(heading);
    inRobotFrame(1, 1) = std::cos(heading);
    return K * inRobotFrame * (r - x);
}

Eigen::Matrix<double, 5, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    auto motors = frc::DCMotor::CIM(2);

    // constexpr double Glow = 15.32;       // Low gear ratio
    constexpr double Ghigh = 7.08;       // High gear ratio
    constexpr auto rb = 0.8382_m / 2.0;  // Robot radius
    constexpr auto r = 0.0746125_m;      // Wheel radius
    constexpr auto m = 63.503_kg;        // Robot mass
    constexpr auto J = 5.6_kg_sq_m;      // Robot moment of inertia

    auto C1 = -std::pow(Ghigh, 2) * motors.Kt /
              (motors.Kv * motors.R * units::math::pow<2>(r));
    auto C2 = Ghigh * motors.Kt / (motors.R * r);
    auto k1 = (1 / m + units::math::pow<2>(rb) / J);
    auto k2 = (1 / m - units::math::pow<2>(rb) / J);

    units::meters_per_second_t vl{x(3, 0)};
    units::meters_per_second_t vr{x(4, 0)};
    units::volt_t Vl{u(0, 0)};
    units::volt_t Vr{u(1, 0)};

    Eigen::Matrix<double, 5, 1> result;
    auto v = 0.5 * (vl + vr);
    result(0, 0) = v.to<double>() * std::cos(x(2, 0));
    result(1, 0) = v.to<double>() * std::sin(x(2, 0));
    result(2, 0) = ((vr - vl) / (2.0 * rb)).to<double>();
    result(3, 0) =
        k1.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
        k2.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>());
    result(4, 0) =
        k2.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
        k1.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>());
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
    return x;
}

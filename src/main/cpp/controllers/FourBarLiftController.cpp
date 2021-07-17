// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/FourBarLiftController.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::FourBarLift;

FourBarLiftController::FourBarLiftController() { Reset(); }

void FourBarLiftController::SetGoal(units::radian_t goal) {
    m_goal = {goal, 0_rad_per_s};
}

void FourBarLiftController::SetReferences(
    units::radian_t angle, units::radians_per_second_t velocity) {
    m_nextR << angle.to<double>(), velocity.to<double>();
}

bool FourBarLiftController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void FourBarLiftController::SetClimbing(bool climbing) {
    m_climbing = climbing;
}

void FourBarLiftController::Reset() {
    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Matrix<double, 1, 1> FourBarLiftController::Calculate(
    const Eigen::Matrix<double, 2, 1>& x) {
    frc::TrapezoidProfile<units::radians>::State references = {
        units::radian_t(m_nextR(0)), units::radians_per_second_t(m_nextR(1))};
    frc::TrapezoidProfile<units::radians> profile{constraints, m_goal,
                                                  references};
    m_profiledReference =
        profile.Calculate(RealTimeRobot::kDefaultControllerPeriod);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_u = m_lqr.Calculate(x, m_r) + m_ff.Calculate(m_nextR);

    m_u = frc::NormalizeInputVector<1>(m_u, 12.0);
    m_r = m_nextR;

    UpdateAtReferences(m_nextR - x);

    // Feedforward compensates for unmodeled extra weight from lifting robot
    // while climbing
    if (m_climbing) {
        Eigen::Matrix<double, 1, 1> extraWeight{2.0};
        return m_u -= extraWeight;
    } else {
        return m_u;
    }
}

frc::LinearSystem<2, 1, 1> FourBarLiftController::GetPlant() {
    // Arm moment of inertia
    constexpr auto J = 0.6975_kg_sq_m;

    return frc::LinearSystemId::SingleJointedArmSystem(
        frc::DCMotor::NEO(), J, Constants::FourBarLift::kGearRatio);
}

void FourBarLiftController::UpdateAtReferences(
    const Eigen::Matrix<double, 2, 1>& error) {
    m_atReferences =
        std::abs(error(0, 0)) < FourBarLiftController::kAngleTolerance &&
        std::abs(error(1, 0)) <
            FourBarLiftController::kAngularVelocityTolerance;
}

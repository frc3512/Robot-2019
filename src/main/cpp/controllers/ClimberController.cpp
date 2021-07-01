// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/ClimberController.hpp"

#include <cmath>

#include <wpi/raw_ostream.h>

using namespace frc3512;

ClimberController::ClimberController() { Reset(); }

void ClimberController::SetGoal(units::meter_t goal) { m_goal = {goal, 0_mps}; }

bool ClimberController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void ClimberController::SetReferences(units::meter_t position,
                                      units::meters_per_second_t velocity) {
    m_nextR << position.to<double>(), velocity.to<double>();
}

void ClimberController::Reset() {
    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Matrix<double, 1, 1> ClimberController::Calculate(
    const Eigen::Matrix<double, 2, 1>& x) {
    frc::TrapezoidProfile<units::meters>::State references = {
        units::meter_t(m_nextR(0)), units::meters_per_second_t(m_nextR(1))};
    frc::TrapezoidProfile<units::meters> profile{constraints, m_goal,
                                                 references};
    m_profiledReference =
        profile.Calculate(RealTimeRobot::kDefaultControllerPeriod);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_u = m_lqr.Calculate(x, m_r) + m_ff.Calculate(m_nextR);

    m_u = frc::NormalizeInputVector<1>(m_u, 12.0);
    m_r = m_nextR;

    UpdateAtReferences(m_nextR - x);

    return m_u;
}

frc::LinearSystem<2, 1, 1> ClimberController::GetPlant() {
    // Radius of axle
    constexpr auto r = 0.003175_m;

    return frc::LinearSystemId::ElevatorSystem(
        frc::DCMotor::Vex775Pro(),
        units::kilogram_t{Constants::Climber::kRobotMass}, r,
        Constants::Climber::kGearRatio);
}

void ClimberController::UpdateAtReferences(
    const Eigen::Matrix<double, 2, 1>& error) {
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;
}

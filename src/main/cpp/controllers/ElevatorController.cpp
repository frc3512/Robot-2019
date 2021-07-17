// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/ElevatorController.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::Elevator;

ElevatorController::ElevatorController() { Reset(); }

void ElevatorController::SetScoringIndex() {
    m_activeConstraints = scoringConstraints;
    m_climbing = false;
}

void ElevatorController::SetClimbingIndex() {
    m_activeConstraints = climbingConstraints;
    m_climbing = true;
}

bool ElevatorController::IsClimbing() const { return m_climbing; }

void ElevatorController::SetGoal(double goal) {
    m_goal = {units::meter_t{goal}, 0_mps};
}

void ElevatorController::SetReferences(units::meter_t position,
                                       units::meters_per_second_t velocity) {
    double positionRef = position.to<double>();
    double velocityRef = velocity.to<double>();
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << positionRef, velocityRef;
    m_nextR = nextR;
}

bool ElevatorController::AtReferences() const { return m_atReferences; }

bool ElevatorController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void ElevatorController::Reset() {
    m_scoreLQR.Reset();
    m_climbLQR.Reset();
    m_nextR.setZero();
    m_u.setZero();
}

Eigen::Matrix<double, 1, 1> ElevatorController::Calculate(
    const Eigen::Matrix<double, 2, 1>& x) {
    frc::TrapezoidProfile<units::meters>::State references = {
        units::meter_t(m_nextR(0, 0)),
        units::meters_per_second_t(m_nextR(1, 0))};
    frc::TrapezoidProfile<units::meters> profile{m_activeConstraints, m_goal,
                                                 references};
    m_profiledReference =
        profile.Calculate(RealTimeRobot::kDefaultControllerPeriod);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    auto& lqr = m_climbing ? m_climbLQR : m_scoreLQR;
    auto& ff = m_climbing ? m_climbFF : m_scoreFF;

    m_u = lqr.Calculate(x, m_r) + ff.Calculate(m_nextR);

    // Feedforward compensates for unmodeled extra weight from lifting robot
    // while climbing
    if (m_climbing) {
        Eigen::Matrix<double, 1, 1> extraWeight{1.0};
        return m_u -= extraWeight;
    } else {
        return m_u;
    }
}

frc::LinearSystem<2, 1, 1> ElevatorController::GetScorePlant() {
    // Radius of pulley
    constexpr auto r = 0.0181864_m;
    // Carriage mass
    constexpr auto m = 9.785262_kg;

    return frc::LinearSystemId::ElevatorSystem(
        frc::DCMotor::NEO(), m, r, Constants::Elevator::kScoringGearRatio);
}

frc::LinearSystem<2, 1, 1> ElevatorController::GetClimbPlant() {
    // Carriage mass
    constexpr auto m = 8.381376_kg;

    // Radius of pulley
    constexpr auto r = 0.0181864_m;

    return frc::LinearSystemId::ElevatorSystem(
        frc::DCMotor::NEO(), m, r, Constants::Elevator::kClimbingGearRatio);
}

void ElevatorController::UpdateAtReferences(
    const Eigen::Matrix<double, 2, 1>& error) {}

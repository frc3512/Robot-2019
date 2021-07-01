// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/ElevatorController.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::Elevator;

ElevatorController::ElevatorController() { m_y.setZero(); }

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

void ElevatorController::SetMeasuredPosition(double measuredPosition) {
    m_y(0, 0) = measuredPosition;
}

double ElevatorController::ControllerVoltage() const {
    if (m_climbing) {
        // Feedforward compensates for unmodeled extra weight from lifting robot
        // while climbing
        return m_climbController.U(0) - 1.0;
    } else {
        return m_scoreController.U(0);
    }
}

double ElevatorController::EstimatedPosition() const {
    const auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;
    return observer.Xhat(0);
}

double ElevatorController::EstimatedVelocity() const {
    const auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;
    return observer.Xhat(1);
}

double ElevatorController::PositionError() const {
    const auto& controller = m_climbing ? m_climbController : m_scoreController;
    const auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;
    return controller.R(0) - observer.Xhat(0);
}

double ElevatorController::VelocityError() const {
    const auto& controller = m_climbing ? m_climbController : m_scoreController;
    const auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;
    return controller.R(1) - observer.Xhat(1);
}

double ElevatorController::PositionReference() {
    return m_profiledReference.position.to<double>();
}

double ElevatorController::VelocityReference() {
    return m_profiledReference.velocity.to<double>();
}

void ElevatorController::Update() {
    elevatorLogger.Log(EstimatedPosition(), EstimatedVelocity(),
                       PositionReference(), ControllerVoltage(),
                       VelocityReference());

    frc::TrapezoidProfile<units::meters>::State references = {
        units::meter_t(m_nextR(0, 0)),
        units::meters_per_second_t(m_nextR(1, 0))};
    frc::TrapezoidProfile<units::meters> profile{m_activeConstraints, m_goal,
                                                 references};
    m_profiledReference =
        profile.Calculate(RealTimeRobot::kDefaultControllerPeriod);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    auto& controller = m_climbing ? m_climbController : m_scoreController;
    auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;

    observer.Correct(m_u, m_y);

    auto error = controller.R() - observer.Xhat();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_u = controller.Calculate(observer.Xhat(), m_nextR);
    observer.Predict(m_u, RealTimeRobot::kDefaultControllerPeriod);
}

void ElevatorController::Reset() {
    m_scoreController.Reset();
    m_climbController.Reset();
    m_scoreObserver.Reset();
    m_climbObserver.Reset();
    m_nextR.setZero();
    m_u.setZero();
}

Eigen::Matrix<double, 1, 1> ElevatorController::Calculate(
    const Eigen::Matrix<double, 2, 1>& x) {
    return Eigen::Matrix<double, 1, 1>::Zero();
}

frc::LinearSystem<2, 1, 1> ElevatorController::GetPlant(bool climbing) {
    auto motor = frc::DCMotor::NEO();

    // Radius of pulley
    constexpr auto r = 0.0181864_m;
    if (climbing) {
        // Carriage mass
        constexpr auto m = 8.381376_kg;

        return frc::LinearSystemId::ElevatorSystem(
            motor, m, r, Constants::Elevator::kClimbingGearRatio);
    } else {
        // Carriage mass
        constexpr auto m = 9.785262_kg;

        return frc::LinearSystemId::ElevatorSystem(
            motor, m, r, Constants::Elevator::kScoringGearRatio);
    }
}

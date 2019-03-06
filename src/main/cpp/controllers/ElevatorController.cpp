// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "controllers/ElevatorController.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::Elevator;

ElevatorController::ElevatorController() { m_y.setZero(); }

void ElevatorController::Enable() { m_isEnabled = true; }

void ElevatorController::Disable() { m_isEnabled = false; }

bool ElevatorController::IsEnabled() const { return m_isEnabled; }

void ElevatorController::SetScoringIndex() {
    m_activeConstraints = scoringConstraints;
    m_climbing = false;
}

void ElevatorController::SetClimbingIndex() {
    m_activeConstraints = climbingConstraints;
    m_climbing = true;
}

void ElevatorController::SetGoal(double goal) {
    m_positionProfile =
        frc::TrapezoidProfile{m_activeConstraints,
                              {units::meter_t{goal}, 0_mps},
                              {units::meter_t{EstimatedPosition()}, 0_mps}};
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

    frc::TrapezoidProfile::State references = {
        units::meter_t(m_nextR(0, 0)),
        units::meters_per_second_t(m_nextR(1, 0))};
    frc::TrapezoidProfile profile{m_activeConstraints, m_goal, references};
    m_profiledReference = profile.Calculate(Constants::kDt);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    auto& controller = m_climbing ? m_climbController : m_scoreController;
    auto& observer = m_climbing ? m_climbObserver : m_scoreObserver;

    observer.Correct(controller.U(), m_y);

    auto error = controller.R() - observer.Xhat();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    controller.Update(observer.Xhat(), m_nextR);
    observer.Predict(controller.U(), Constants::kDt);
}

void ElevatorController::Reset() {
    m_scorePlant.Reset();
    m_climbPlant.Reset();
    m_scoreController.Reset();
    m_climbController.Reset();
    m_scoreObserver.Reset();
    m_climbObserver.Reset();
    m_nextR.setZero();
}

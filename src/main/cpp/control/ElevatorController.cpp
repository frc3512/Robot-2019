// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/ElevatorController.hpp"

#include <cmath>

#include "Robot.hpp"

using namespace frc3512;

ElevatorController::ElevatorController() { m_Y.setZero(); }

void ElevatorController::Enable() { m_controller.Enable(); }

void ElevatorController::Disable() { m_controller.Disable(); }

void ElevatorController::SetScoringIndex() {
    m_controller.SetIndex(0);
    m_observer.SetIndex(0);
    m_plant.SetIndex(0);
    m_activeConstraints = scoringConstraints;
}

void ElevatorController::SetClimbingIndex() {
    m_controller.SetIndex(1);
    m_observer.SetIndex(1);
    m_plant.SetIndex(1);
    m_activeConstraints = climbingConstraints;
}

void ElevatorController::SetGoal(double goal) {
    m_positionProfile =
        TrapezoidalMotionProfile{m_activeConstraints,
                                 {units::meter_t{goal}, 0_mps},
                                 {units::meter_t{EstimatedPosition()}, 0_mps}};
    m_goal = {units::meter_t{goal}, 0_mps};
}

void ElevatorController::SetReferences(units::meter_t position,
                                       units::meters_per_second_t velocity) {
    double positionRef = units::unit_cast<double>(position);
    double velocityRef = units::unit_cast<double>(velocity);
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << positionRef, velocityRef;
    m_nextR = nextR;
}

bool ElevatorController::AtReferences() const { return m_atReferences; }

bool ElevatorController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void ElevatorController::SetMeasuredPosition(double measuredPosition) {
    m_Y(0, 0) = measuredPosition;
}

double ElevatorController::ControllerVoltage() const {
    if (m_controller.GetIndex() == 1) {
        // Feedforward compensates for unmodeled extra weight from lifting robot
        // while climbing
        return m_controller.U(0) - 1.0;
    } else {
        return m_controller.U(0) +
               (std::pow(kCarriageMass, 2) * kGravity * kResistance *
                kDrumRadius / (kElevatorGearRatio * kKt));
    }
}

double ElevatorController::EstimatedPosition() const {
    return m_observer.Xhat(0);
}

double ElevatorController::EstimatedVelocity() const {
    return m_observer.Xhat(1);
}

double ElevatorController::PositionError() const {
    return m_controller.R(0) - m_observer.Xhat(0);
}

double ElevatorController::VelocityError() const {
    return m_controller.R(1) - m_observer.Xhat(1);
}

double ElevatorController::PositionReference() {
    double positionRef = units::unit_cast<double>(m_profiledReference.position);
    return positionRef;
}

void ElevatorController::Update() {
    elevatorLogger.Log(EstimatedPosition(), PositionReference(),
                       ControllerVoltage());

    TrapezoidalMotionProfile::State references = {
        units::meter_t(m_nextR(0, 0)),
        units::meters_per_second_t(m_nextR(1, 0))};
    TrapezoidalMotionProfile profile{m_activeConstraints, m_goal, references};
    m_profiledReference = profile.Calculate(kDt_s);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_observer.Correct(m_controller.U(), m_Y);

    auto error = m_controller.R() - m_observer.Xhat();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_observer.Predict(m_controller.U());
    m_controller.Update(m_nextR, m_observer.Xhat());
}

void ElevatorController::Reset() {
    m_plant.Reset();
    m_controller.Reset();
    m_observer.Reset();
    m_nextR.setZero();
}

void ElevatorController::SetClimbingProfile() {
    m_activeConstraints = climbingConstraints;
    m_plant.SetIndex(1);
    m_controller.SetIndex(1);
    m_observer.SetIndex(1);
}

void ElevatorController::SetScoringProfile() {
    m_activeConstraints = scoringConstraints;
    m_plant.SetIndex(0);
    m_controller.SetIndex(0);
    m_observer.SetIndex(0);
}

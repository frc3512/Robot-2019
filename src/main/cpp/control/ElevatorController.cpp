// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/ElevatorController.hpp"

#include <cmath>

#include "Robot.hpp"

using namespace frc3512;

ElevatorController::ElevatorController() { m_Y.setZero(); }

void ElevatorController::Enable() { m_loop.Enable(); }

void ElevatorController::Disable() { m_loop.Disable(); }

void ElevatorController::SetGoal(double goal) {
    m_positionProfile =
        TrapezoidalMotionProfile{constraints,
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
    m_loop.SetNextR(nextR);
}

bool ElevatorController::AtReferences() const { return m_atReferences; }

void ElevatorController::SetMeasuredPosition(double measuredPosition) {
    m_Y(0, 0) = measuredPosition;
}

double ElevatorController::ControllerVoltage() const {
    return m_loop.U(0) + (std::pow(kCarriageMass, 2) * kGravity * kResistance *
                          kDrumRadius / (kElevatorGearRatio * kKt));
}

double ElevatorController::EstimatedPosition() const { return m_loop.Xhat(0); }

double ElevatorController::EstimatedVelocity() const { return m_loop.Xhat(1); }

double ElevatorController::PositionError() const {
    return m_loop.Error()(0, 0);
}

double ElevatorController::VelocityError() const {
    return m_loop.Error()(1, 0);
}

double ElevatorController::PositionReference() {
    double positionRef = units::unit_cast<double>(m_profiledReference.position);
    return positionRef;
}

void ElevatorController::Update() {
    elevatorLogger.Log(EstimatedPosition(), PositionReference(),
                       ControllerVoltage());

    TrapezoidalMotionProfile::State references = {
        units::meter_t(m_loop.NextR(0)),
        units::meters_per_second_t(m_loop.NextR(1))};
    TrapezoidalMotionProfile profile{constraints, m_goal, references};
    m_profiledReference = profile.Calculate(kDt_s);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_loop.Correct(m_Y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_loop.Predict();
}

void ElevatorController::Reset() { m_loop.Reset(); }

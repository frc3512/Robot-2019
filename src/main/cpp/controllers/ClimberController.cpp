// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/ClimberController.hpp"

#include <cmath>

#include <wpi/raw_ostream.h>

using namespace frc3512;

ClimberController::ClimberController() { m_y.setZero(); }

void ClimberController::Enable() { m_isEnabled = true; }

void ClimberController::Disable() { m_isEnabled = false; }

void ClimberController::SetGoal(double goal) {
    m_positionProfile = frc::TrapezoidProfile<units::meters>{
        constraints,
        {units::meter_t{goal}, 0_mps},
        {units::meter_t{EstimatedPosition()}, 0_mps}};
    m_goal = {units::meter_t{goal}, 0_mps};
}

void ClimberController::SetReferences(units::meter_t position,
                                      units::meters_per_second_t velocity) {
    double positionRef = position.to<double>();
    double velocityRef = velocity.to<double>();
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << positionRef, velocityRef;
    m_loop.SetNextR(nextR);
}

bool ClimberController::AtReferences() const { return m_atReferences; }

bool ClimberController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void ClimberController::SetMeasuredPosition(double measuredPosition) {
    m_y(0, 0) = measuredPosition;
}

double ClimberController::ControllerVoltage() { return m_loop.U(0); }

double ClimberController::EstimatedPosition() const { return m_loop.Xhat(0); }

double ClimberController::EstimatedVelocity() const { return m_loop.Xhat(1); }

double ClimberController::PositionError() const { return m_loop.Error()(0, 0); }

double ClimberController::VelocityError() const { return m_loop.Error()(1, 0); }

double ClimberController::PositionReference() {
    return m_profiledReference.position.to<double>();
}

double ClimberController::VelocityReference() {
    return m_profiledReference.velocity.to<double>();
}

void ClimberController::Update() {
    climberLogger.Log(EstimatedPosition(), PositionReference(),
                      ControllerVoltage(), EstimatedVelocity(),
                      VelocityReference());

    frc::TrapezoidProfile<units::meters>::State references = {
        units::meter_t(m_loop.NextR(0)),
        units::meters_per_second_t(m_loop.NextR(1))};
    frc::TrapezoidProfile<units::meters> profile{constraints, m_goal,
                                                 references};
    m_profiledReference = profile.Calculate(Constants::kDt);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_loop.Correct(m_y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_loop.Predict(Constants::kDt);
}

void ClimberController::Reset() {
    m_loop.Reset(Eigen::Matrix<double, 2, 1>::Zero());
}

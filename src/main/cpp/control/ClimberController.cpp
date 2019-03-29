// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/ClimberController.hpp"

#include <cmath>

#include <wpi/raw_ostream.h>

#include "Robot.hpp"

using namespace frc3512;

ClimberController::ClimberController() { m_Y.setZero(); }

void ClimberController::Enable() { m_loop.Enable(); }

void ClimberController::Disable() { m_loop.Disable(); }

void ClimberController::SetGoal(double goal) {
    m_positionProfile =
        TrapezoidalMotionProfile{constraints,
                                 {units::meter_t{goal}, 0_mps},
                                 {units::meter_t{EstimatedPosition()}, 0_mps}};
    m_goal = {units::meter_t{goal}, 0_mps};
}

void ClimberController::SetReferences(units::meter_t position,
                                      units::meters_per_second_t velocity) {
    double positionRef = units::unit_cast<double>(position);
    double velocityRef = units::unit_cast<double>(velocity);
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << positionRef, velocityRef;
    m_loop.SetNextR(nextR);
}

bool ClimberController::AtReferences() const { return m_atReferences; }

bool ClimberController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

bool ClimberController::ErrorExceeded() const { return m_errorExceeded; }

void ClimberController::SetMeasuredPosition(double measuredPosition) {
    m_Y(0, 0) = measuredPosition;
}

double ClimberController::ControllerVoltage() {
    /*constexpr double kFeedForward = kRobotMass* kRobotMass * kGravity *
       kResistance * kSprocketRadius / (kClimberGearRatio * kKt);*/
    constexpr double kFeedForward = 0;
    return m_loop.U(0) + kFeedForward;
}

double ClimberController::EstimatedPosition() const { return m_loop.Xhat(0); }

double ClimberController::EstimatedVelocity() const { return m_loop.Xhat(1); }

double ClimberController::PositionError() const { return m_loop.Error()(0, 0); }

double ClimberController::VelocityError() const { return m_loop.Error()(1, 0); }

double ClimberController::PositionReference() {
    double positionRef = units::unit_cast<double>(m_profiledReference.position);
    return positionRef;
}

void ClimberController::Update() {
    climberLogger.Log(EstimatedPosition(), PositionReference(),
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

void ClimberController::Reset() { m_loop.Reset(); }

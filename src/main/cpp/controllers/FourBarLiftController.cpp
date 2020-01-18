// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/FourBarLiftController.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::FourBarLift;

FourBarLiftController::FourBarLiftController() { m_y.setZero(); }

void FourBarLiftController::Enable() { m_isEnabled = true; }

void FourBarLiftController::Disable() { m_isEnabled = false; }

void FourBarLiftController::SetGoal(double goal) {
    m_angleProfile = frc::TrapezoidProfile<units::radians>{
        constraints,
        {units::radian_t{goal}, 0_rad_per_s},
        {units::radian_t{GetStates()(0)}, 0_rad_per_s}};
    m_goal = {units::radian_t{goal}, 0_rad_per_s};
}

void FourBarLiftController::SetReferences(
    units::radian_t angle, units::radians_per_second_t velocity) {
    double angleRef = units::unit_cast<double>(angle);
    double velocityRef = units::unit_cast<double>(velocity);
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << angleRef, velocityRef;
    m_loop.SetNextR(nextR);
}

bool FourBarLiftController::AtReferences() const { return m_atReferences; }

bool FourBarLiftController::AtGoal() const {
    return m_atReferences && m_goal == m_profiledReference;
}

void FourBarLiftController::SetMeasuredAngle(double measuredAngle) {
    m_y(0) = measuredAngle;
}

void FourBarLiftController::SetClimbing(bool climbing) {
    m_climbing = climbing;
}

double FourBarLiftController::AngleError() const { return m_loop.Error()(0); }

double FourBarLiftController::AngularVelocityError() const {
    return m_loop.Error()(1);
}

const Eigen::Matrix<double, 2, 1>& FourBarLiftController::GetReferences()
    const {
    return m_loop.NextR();
}

const Eigen::Matrix<double, 2, 1>& FourBarLiftController::GetStates() const {
    return m_loop.Xhat();
}

const Eigen::Matrix<double, 1, 1>& FourBarLiftController::GetInputs() const {
    if (!m_climbing) {
        m_u = m_loop.U();
        return m_u;
    } else {
        // Feedforward compensates for unmodeled extra weight from lifting robot
        // while climbing
        m_u = m_loop.U();
        m_u(0) -= 2.0;
        return m_u;
    }
}

const Eigen::Matrix<double, 1, 1>& FourBarLiftController::GetOutputs() const {
    return m_y;
}

void FourBarLiftController::Update() {
    m_logger.Log(GetReferences(), GetStates(), GetInputs(), GetOutputs());

    frc::TrapezoidProfile<units::radians>::State references = {
        units::radian_t(m_loop.NextR(0)),
        units::radians_per_second_t(m_loop.NextR(1))};
    frc::TrapezoidProfile<units::radians> profile{constraints, m_goal,
                                                  references};
    m_profiledReference = profile.Calculate(Constants::kDt);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_loop.Correct(m_y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0)) < kAngleTolerance &&
                     std::abs(error(1)) < kAngularVelocityTolerance;

    m_loop.Predict(Constants::kDt);
}

void FourBarLiftController::Reset() {
    m_loop.Reset(Eigen::Matrix<double, 2, 1>::Zero());
    m_u.setZero();
}

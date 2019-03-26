// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/FourBarLiftController.hpp"

#include <cmath>

#include "Robot.hpp"

using namespace frc3512;

FourBarLiftController::FourBarLiftController() { m_Y.setZero(); }

void FourBarLiftController::Enable() { m_loop.Enable(); }

void FourBarLiftController::Disable() { m_loop.Disable(); }

void FourBarLiftController::SetGoal(double goal) {
    m_angleProfile =
        TrapezoidalMotionProfile{constraints,
                                 {units::meter_t{goal}, 0_mps},
                                 {units::meter_t{EstimatedAngle()}, 0_mps}};
    m_goal = {units::meter_t{goal}, 0_mps};
}

void FourBarLiftController::SetReferences(units::meter_t angle,
                                          units::meters_per_second_t velocity) {
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
    m_Y(0, 0) = measuredAngle;
}

double FourBarLiftController::ControllerVoltage() const {
    if (!m_climbing) {
        return m_loop.U(0) +
               (1 / 2) *
                   (kRobotVoltage * kFourBarLiftMass * kGravity *
                    kFourBarLiftLength) /
                   (kFourBarLiftGearRatio *
                    (kFourBarLiftStallTorque / kFourBarLiftStallCurrent)) *
                   std::cos(EstimatedAngle() + 1.5);
    } else {
        // Feedforward compensates for unmodeled extra weight from lifting robot
        // while climbing
        return m_loop.U(0) - 2.0;
    }
}

void FourBarLiftController::SetClimbing(bool climbing) {
    m_climbing = climbing;
}

double FourBarLiftController::EstimatedAngle() const { return m_loop.Xhat(0); }

double FourBarLiftController::EstimatedAngularVelocity() const {
    return m_loop.Xhat(1);
}

double FourBarLiftController::AngleError() const {
    return m_loop.Error()(0, 0);
}

double FourBarLiftController::AngularVelocityError() const {
    return m_loop.Error()(1, 0);
}

double FourBarLiftController::AngleReference() {
    double angleRef = units::unit_cast<double>(m_profiledReference.position);
    return angleRef;
}

void FourBarLiftController::Update() {
    elevatorLogger.Log(EstimatedAngle(), AngleReference(), ControllerVoltage());

    TrapezoidalMotionProfile::State references = {
        units::meter_t(m_loop.NextR(0)),
        units::meters_per_second_t(m_loop.NextR(1))};
    TrapezoidalMotionProfile profile{constraints, m_goal, references};
    m_profiledReference = profile.Calculate(kDt_s);

    SetReferences(m_profiledReference.position, m_profiledReference.velocity);

    m_loop.Correct(m_Y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kAngleTolerance &&
                     std::abs(error(1, 0)) < kAngularVelocityTolerance;

    m_loop.Predict();
}

void FourBarLiftController::Reset() { m_loop.Reset(); }

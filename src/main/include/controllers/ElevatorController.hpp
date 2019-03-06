// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/ElevatorSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.hpp"
#include "logging/CsvLogger.hpp"

namespace frc3512 {

class ElevatorController {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    ElevatorController();

    ElevatorController(const ElevatorController&) = delete;
    ElevatorController& operator=(const ElevatorController&) = delete;

    void Enable();
    void Disable();
    bool IsEnabled() const;

    void SetScoringIndex();
    void SetClimbingIndex();

    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param position Position of the carriage in meters.
     * @param velocity Velocity of the carriage in meters per second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t velocity);

    bool AtReferences() const;

    bool AtGoal() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredPosition Position of the carriage in meters.
     */
    void SetMeasuredPosition(double measuredPosition);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage() const;

    /**
     * Returns the estimated position.
     */
    double EstimatedPosition() const;

    /**
     * Returns the estimated velocity.
     */
    double EstimatedVelocity() const;

    /**
     * Returns the error between the position reference and the position
     * estimate.
     */
    double PositionError() const;

    /**
     * Returns the error between the velocity reference and the velocity
     * estimate.
     */
    double VelocityError() const;

    /**
     * Returns the current position reference set by the profile.
     */
    double PositionReference();

    /**
     * Returns the current velocity reference set by the profile.
     */
    double VelocityReference();

    /**
     * Executes the control loop for a cycle.
     */
    void Update();

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile::State m_goal;

    frc::TrapezoidProfile::Constraints scoringConstraints{
        Constants::Elevator::kMaxV, Constants::Elevator::kMaxA};
    frc::TrapezoidProfile::Constraints climbingConstraints{
        Constants::Elevator::kClimbMaxV, Constants::Elevator::kClimbMaxA};
    frc::TrapezoidProfile::Constraints m_activeConstraints = scoringConstraints;

    frc::TrapezoidProfile m_positionProfile{scoringConstraints, {0_m, 0_mps}};

    frc::TrapezoidProfile::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_scorePlant = [=] {
        auto motor = frc::DCMotor::NEO();

        // Carriage mass
        constexpr auto m = 9.785262_kg;

        // Radius of pulley
        constexpr auto r = 0.0181864_m;

        // Gear ratio
        constexpr double G = 8.0;

        return frc::ElevatorSystem(motor, m, r, G);
    }();

    frc::LinearSystem<2, 1, 1> m_climbPlant = [=] {
        auto motor = frc::DCMotor::NEO();

        // Carriage mass
        constexpr auto m = 8.381376_kg;

        // Radius of pulley
        constexpr auto r = 0.0181864_m;

        // Gear ratio
        constexpr double G = 12.5;

        return frc::ElevatorSystem(motor, m, r, G);
    }();

    frc::LinearQuadraticRegulator<2, 1, 1> m_scoreController{
        m_scorePlant, {0.3, 3.0}, {12.0}, Constants::kDt};
    frc::LinearQuadraticRegulator<2, 1, 1> m_climbController{
        m_climbPlant, {0.3, 3.0}, {12.0}, Constants::kDt};

    frc::KalmanFilter<2, 1, 1> m_scoreObserver{
        m_scorePlant, {0.05, 100.0}, {0.0001}};
    frc::KalmanFilter<2, 1, 1> m_climbObserver{
        m_climbPlant, {0.05, 100.0}, {0.0001}};

    bool m_isEnabled = false;
    bool m_climbing = false;

    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;

    CsvLogger elevatorLogger{"/home/lvuser/Elevator.csv",
                             "Time (s),EstPos (m),EstVel (m/s),RefPos "
                             "(m),Voltage (V),RefVel (m/s)"};
};

}  // namespace frc3512

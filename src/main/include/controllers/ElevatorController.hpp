// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class ElevatorController : public ControllerBase<2, 1, 1> {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    class State {
    public:
        static constexpr int kPosition = 0;
        static constexpr int kVelocity = 1;
    };

    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    class Output {
    public:
        static constexpr int kPosition = 0;
    };

    ElevatorController();

    ElevatorController(const ElevatorController&) = delete;
    ElevatorController& operator=(const ElevatorController&) = delete;

    void SetScoringIndex();
    void SetClimbingIndex();

    bool IsClimbing() const;

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

    Eigen::Matrix<double, 1, 1> Calculate(
        const Eigen::Matrix<double, 2, 1>& x) override;

    /**
     * Returns the elevator plant.
     *
     * @param climbing Returns climbing plant if true and scoring plant if false
     */
    static frc::LinearSystem<2, 1, 1> GetPlant(bool climbing);

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile<units::meters>::State m_goal;

    frc::TrapezoidProfile<units::meters>::Constraints scoringConstraints{
        Constants::Elevator::kMaxV, Constants::Elevator::kMaxA};
    frc::TrapezoidProfile<units::meters>::Constraints climbingConstraints{
        Constants::Elevator::kClimbMaxV, Constants::Elevator::kClimbMaxA};
    frc::TrapezoidProfile<units::meters>::Constraints m_activeConstraints =
        scoringConstraints;

    frc::TrapezoidProfile<units::meters>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_scorePlant = GetPlant(false);
    frc::LinearSystem<2, 1, 1> m_climbPlant = GetPlant(true);

    frc::LinearQuadraticRegulator<2, 1> m_scoreController{
        m_scorePlant,
        {0.3, 3.0},
        {12.0},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearQuadraticRegulator<2, 1> m_climbController{
        m_climbPlant,
        {0.3, 3.0},
        {12.0},
        RealTimeRobot::kDefaultControllerPeriod};

    frc::KalmanFilter<2, 1, 1> m_scoreObserver{
        m_scorePlant,
        {0.05, 100.0},
        {0.0001},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::KalmanFilter<2, 1, 1> m_climbObserver{
        m_climbPlant,
        {0.05, 100.0},
        {0.0001},
        RealTimeRobot::kDefaultControllerPeriod};

    bool m_climbing = false;

    bool m_atReferences = false;

    frc::CSVLogFile elevatorLogger{"Elevator",   "EstPos (m)",  "EstVel (m/s)",
                                   "RefPos (m)", "Voltage (V)", "RefVel (m/s)"};
};

}  // namespace frc3512

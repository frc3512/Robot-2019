// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearPlantInversionFeedforward.h>
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
     * Resets any internal state.
     */
    void Reset();

    Eigen::Matrix<double, 1, 1> Calculate(
        const Eigen::Matrix<double, 2, 1>& x) override;

    /**
     * Returns the elevator score plant.
     */
    static frc::LinearSystem<2, 1, 1> GetScorePlant();

    /**
     * Returns the elevator climb plant.
     */
    static frc::LinearSystem<2, 1, 1> GetClimbPlant();

private:
    frc::TrapezoidProfile<units::meters>::State m_goal;

    frc::TrapezoidProfile<units::meters>::Constraints scoringConstraints{
        Constants::Elevator::kMaxV, Constants::Elevator::kMaxA};
    frc::TrapezoidProfile<units::meters>::Constraints climbingConstraints{
        Constants::Elevator::kClimbMaxV, Constants::Elevator::kClimbMaxA};
    frc::TrapezoidProfile<units::meters>::Constraints m_activeConstraints =
        scoringConstraints;

    frc::TrapezoidProfile<units::meters>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_scorePlant = GetScorePlant();
    frc::LinearSystem<2, 1, 1> m_climbPlant = GetClimbPlant();

    frc::LinearQuadraticRegulator<2, 1> m_scoreLQR{
        m_scorePlant,
        {0.3, 3.0},
        {12.0},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearQuadraticRegulator<2, 1> m_climbLQR{
        m_climbPlant,
        {0.3, 3.0},
        {12.0},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_scoreFF{
        m_scorePlant, RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_climbFF{
        m_climbPlant, RealTimeRobot::kDefaultControllerPeriod};

    bool m_climbing = false;

    bool m_atReferences = false;

    void UpdateAtReferences(const Eigen::Matrix<double, 2, 1>& error);
};

}  // namespace frc3512

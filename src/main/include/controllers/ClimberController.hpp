// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class ClimberController : public ControllerBase<2, 1, 1> {
public:
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

    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    ClimberController();

    ClimberController(const ClimberController&) = delete;
    ClimberController& operator=(const ClimberController&) = delete;

    /**
     * Sets the end goal of the controller profile.
     *
     * @param goal Position in meters to set the goal to.
     */
    void SetGoal(units::meter_t goal);

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Sets the references.
     *
     * @param position Position of the carriage in meters.
     * @param velocity Velocity of the carriage in meters per second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t velocity);

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredPosition Position of the carriage in meters.
     */
    void SetMeasuredPosition(units::meter_t measuredPosition);

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

    static frc::LinearSystem<2, 1, 1> GetPlant();

private:
    frc::TrapezoidProfile<units::meters>::State m_goal;

    frc::TrapezoidProfile<units::meters>::Constraints constraints{
        Constants::Climber::kMaxV, Constants::Climber::kMaxA};

    // The current references from the profile.
    frc::TrapezoidProfile<units::meters>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant = GetPlant();
    frc::LinearQuadraticRegulator<2, 1> m_lqr{
        m_plant, {0.02, 0.4}, {12.0}, RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_ff{
        m_plant, RealTimeRobot::kDefaultControllerPeriod};

    bool m_atGoal = false;
    bool m_atReferences = false;

    void UpdateAtReferences(const Eigen::Matrix<double, 2, 1>& error);
};
}  // namespace frc3512

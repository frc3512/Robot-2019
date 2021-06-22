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

namespace frc3512 {

class ClimberController {
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
     * Enables the control loop.
     */
    void Enable();

    /**
     * Disables the control loop.
     */
    void Disable();

    /**
     * Sets the end goal of the controller profile.
     *
     * @param goal Position in meters to set the goal to.
     */
    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param position Position of the carriage in meters.
     * @param velocity Velocity of the carriage in meters per second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t velocity);

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Returns whether or not position and velocity are tracking the profile.
     */
    bool AtReferences() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredPosition Position of the carriage in meters.
     */
    void SetMeasuredPosition(double measuredPosition);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage();

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
    frc::TrapezoidProfile<units::meters>::State m_goal;

    frc::TrapezoidProfile<units::meters>::Constraints constraints{
        Constants::Climber::kMaxV, Constants::Climber::kMaxA};
    frc::TrapezoidProfile<units::meters> m_positionProfile{constraints,
                                                           {0_m, 0_mps}};

    // The current references from the profile.
    frc::TrapezoidProfile<units::meters>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant = [=] {
        constexpr auto motor = frc::DCMotor::Vex775Pro();

        // Robot mass
        constexpr auto m = 63.503_kg;

        // Radius of axle
        constexpr auto r = 0.003175_m;

        // Gear ratio
        constexpr double G = 50.0 / 1.0;

        return frc::LinearSystemId::ElevatorSystem(motor, m, r, G);
    }();
    frc::LinearQuadraticRegulator<2, 1> m_controller{
        m_plant, {0.02, 0.4}, {12.0}, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, {0.05, 1.0}, {0.0001}, Constants::kDt};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_controller, m_observer,
                                          12_V, Constants::kDt};

    bool m_isEnabled = false;

    bool m_atReferences = false;

    frc::CSVLogFile climberLogger{"Climber",      "EstPos (m)",
                                  "PosRef (m)",   "Voltage (V)",
                                  "EstVel (m/s)", "VelRef (m/s)"};
};
}  // namespace frc3512

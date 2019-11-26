// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/ElevatorSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.hpp"
#include "logging/CsvLogger.hpp"

namespace frc3512 {

class ClimberController {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    ClimberController();

    ClimberController(const ClimberController&) = delete;
    ClimberController& operator=(const ClimberController&) = delete;

    void Enable();
    void Disable();

    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param position Position of the carriage in meters.
     * @param velocity Velocity of the carriage in meters per second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t velocity);

    bool AtGoal() const;

    bool AtReferences() const;

    bool ErrorExceeded() const;

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

    frc::TrapezoidProfile<units::meters>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant = [=] {
        auto motor = frc::DCMotor::Vex775Pro();

        // Robot mass
        constexpr auto m = 63.503_kg;

        // Radius of axle
        constexpr auto r = 0.003175_m;

        // Gear ratio
        constexpr double G = 50.0 / 1.0;

        return frc::ElevatorSystem(motor, m, r, G);
    }();
    frc::LinearQuadraticRegulator<2, 1, 1> m_controller{
        m_plant, {0.02, 0.4}, {12.0}, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, Constants::kDt, {0.05, 1.0}, {0.0001}};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_controller, m_observer};

    bool m_atReferences = false;
    bool m_errorExceeded = false;

    CsvLogger climberLogger{"/home/lvuser/Climber.csv",
                            "Time,EstPos,PosRef,Voltage,EstVel,VelRef"};
};

}  // namespace frc3512

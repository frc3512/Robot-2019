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
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"

namespace frc3512 {

class FourBarLiftController {
public:
    // State tolerances in radians and radians/sec respectively.
    static constexpr double kAngleTolerance = 0.05;
    static constexpr double kAngularVelocityTolerance = 2.0;

    class State {
    public:
        static constexpr int kAngle = 0;
        static constexpr int kAngularVelocity = 1;
    };

    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    class Output {
    public:
        static constexpr int kAngle = 0;
    };

    FourBarLiftController();

    FourBarLiftController(const FourBarLiftController&) = delete;
    FourBarLiftController& operator=(const FourBarLiftController&) = delete;

    /**
     * Sets the end goal of the controller profile.
     *
     * @param goal Position in meters to set the goal to.
     */
    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param angle  Angle of the carriage in radians.
     * @param angularVelocity  Angular velocity of the carriage in radians per
     *                         second.
     */
    void SetReferences(units::radian_t angle,
                       units::radians_per_second_t angularVelocity);

    /**
     * Returns whether or not position and velocity are tracking the profile.
     */
    bool AtReferences() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredAngle Angle of the carriage in radians.
     */
    void SetMeasuredAngle(double measuredAngle);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage() const;

    /**
     * Informs the controller if to use the climbing feedforward.
     *
     * @param climbing Whether or not to use the climbing feedforward.
     */
    void SetClimbing(bool climbing);

    /**
     * Returns the estimated angle.
     */
    double EstimatedAngle() const;

    /**
     * Returns the estimated angular velocity.
     */
    double EstimatedAngularVelocity() const;

    /**
     * Returns the error between the angle reference and the angle
     * estimate.
     */
    double AngleError() const;

    /**
     * Returns the error between the angular velocity reference and the angular
     * velocity estimate.
     */
    double AngularVelocityError() const;

    /**
     * Returns the current angle reference.
     */
    double AngleReference();

    /**
     * Returns the current angular velocity reference.
     */
    double AngularVelocityReference();

    /**
     * Executes the control loop for a cycle.
     */
    void Update();

    /**
     * Resets any internal state.
     */
    void Reset();

    /**
     * Returns the four bar lift plant.
     */
    static frc::LinearSystem<2, 1, 1> GetPlant();

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile<units::radians>::State m_goal;

    frc::TrapezoidProfile<units::radians>::Constraints constraints{
        Constants::FourBarLift::kMaxV, Constants::FourBarLift::kMaxA};
    frc::TrapezoidProfile<units::radians> m_angleProfile{constraints,
                                                         {0_rad, 0_rad_per_s}};

    frc::TrapezoidProfile<units::radians>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant = GetPlant();
    frc::LinearQuadraticRegulator<2, 1> m_controller{
        m_plant,
        {0.01245, 0.109726},
        {9.0},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant,
        {0.21745, 0.28726},
        {0.01},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::LinearSystemLoop<2, 1, 1> m_loop{
        m_plant, m_controller, m_observer, 12_V,
        RealTimeRobot::kDefaultControllerPeriod};

    bool m_atReferences = false;
    bool m_climbing = false;

    frc::CSVLogFile elevatorLogger{"FourBarLift",    "EstPos (rad)",
                                   "RefPos (rad)",   "Voltage (V)",
                                   "EstVel (rad/s)", "RefVel (rad/s)"};
};

}  // namespace frc3512

// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/SingleJointedArmSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.hpp"
#include "logging/CsvLogger.hpp"

namespace frc3512 {

class FourBarLiftController {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kAngleTolerance = 0.05;
    static constexpr double kAngularVelocityTolerance = 2.0;

    FourBarLiftController();

    FourBarLiftController(const FourBarLiftController&) = delete;
    FourBarLiftController& operator=(const FourBarLiftController&) = delete;

    void Enable();
    void Disable();

    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param angle  Angle of the carriage in radians.
     * @param angularVelocity  Angular velocity of the carriage in radians per
     *                         second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t angularVelocity);

    bool AtReferences() const;

    bool AtGoal() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredAngle Angle of the carriage in meters.
     */
    void SetMeasuredAngle(double measuredAngle);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage() const;

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

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile::State m_goal;

    frc::TrapezoidProfile::Constraints constraints{
        Constants::FourBarLift::kMaxV, Constants::FourBarLift::kMaxA};
    frc::TrapezoidProfile m_angleProfile{constraints, {0_m, 0_mps}};

    frc::TrapezoidProfile::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant = [=] {
        auto motor = frc::DCMotor::NEO();

        // Arm moment of inertia
        auto J = 0.6975_kg_sq_m;

        // Gear ratio
        constexpr double G = 302.22;

        return frc::SingleJointedArmSystem(motor, J, G);
    }();
    frc::LinearQuadraticRegulator<2, 1, 1> m_controller{
        m_plant, {0.01245, 0.109726}, {9.0}, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{m_plant, {0.21745, 0.28726}, {0.01}};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_controller, m_observer};

    bool m_atReferences = false;
    bool m_climbing = false;

    CsvLogger elevatorLogger{"/home/lvuser/FourBarLift.csv",
                             "Time (s),EstPos (rad),RefPos (rad),Voltage "
                             "(V),EstVel (rad/s),RefVel (rad/s)"};
};

}  // namespace frc3512

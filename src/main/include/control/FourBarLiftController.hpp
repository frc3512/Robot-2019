// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/StateSpaceLoop.h>

#include "Constants.hpp"
#include "control/FourBarLiftCoeffs.hpp"
#include "control/TrapezoidalMotionProfile.hpp"
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
     * Returns the current reference set by the profile
     */
    double AngleReference();

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
    Eigen::Matrix<double, 1, 1> m_Y;
    TrapezoidalMotionProfile::State m_goal;

    TrapezoidalMotionProfile::Constraints constraints{kFourBarLiftMaxV,
                                                      kFourBarLiftMaxA};
    TrapezoidalMotionProfile m_angleProfile{constraints, {0_m, 0_mps}};

    TrapezoidalMotionProfile::State m_profiledReference;

    // The control loop.
    frc::StateSpaceLoop<2, 1, 1> m_loop{MakeFourBarLiftLoop()};

    bool m_atReferences = false;
    bool m_climbing = false;

    CsvLogger elevatorLogger{"/home/lvuser/FourBarLift.csv",
                             "Time,EstPos,PosRef,Voltage"};
};

}  // namespace frc3512

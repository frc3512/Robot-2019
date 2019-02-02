// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <grpl/pf.h>
#include <units.h>

#include <array>
#include <tuple>

#include <Eigen/Core>
#include <frc/Timer.h>
#include <frc/controller/StateSpaceLoop.h>

#include "Constants.hpp"
#include "control/DrivetrainCoeffs.hpp"
#include "control/Pathfinder.hpp"
#include "control/Pose.hpp"
#include "control/TrajectoryPoint.hpp"
#include "control/TrapezoidalMotionProfile.hpp"

class DrivetrainController {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    DrivetrainController();

    DrivetrainController(const DrivetrainController&) = delete;
    DrivetrainController& operator=(const DrivetrainController&) = delete;

    void Enable();
    void Disable();

    /**
     * Sets goal pose of drivetrain controller.
     *
     * @param pose The goal pose.
     */
    void SetGoal(const Pose& pose);

    /**
     * Sets goal pose of drivetrain controller.
     *
     * @param pose The goal pose.
     */
    void SetGoal(Pose&& pose);

    /**
     * Returns whether the drivetrain controller is at the goal pose.
     */
    bool AtGoal() const;

    /**
     * Sets the current encoder measurements.
     *
     * @param leftPosition  Velocity of left side in meters.
     * @param rightPosition Velocity of right side in meters.
     * @param heading       Angle of the robot.
     */
    void SetMeasuredStates(double leftVelocity, double rightVelocity,
                           double heading);

    /**
     * Returns the control loop calculated voltage for the left side.
     */
    double ControllerLeftVoltage() const;

    /**
     * Returns the control loop calculated voltage for the left side.
     */
    double ControllerRightVoltage() const;

    /**
     * Returns the estimated left velocity.
     */
    double EstimatedLeftVelocity() const;

    /**
     * Returns the estimated right velocity.
     */
    double EstimatedRightVelocity() const;

    /**
     * Returns the error between the left velocity reference and the left
     * velocity estimate.
     */
    double LeftVelocityError() const;

    /**
     * Returns the error between the right velocity reference and the right
     * velocity estimate.
     */
    double RightVelocityError() const;

    TrajectoryPoint EstimatedPose() const;

    TrajectoryPoint GoalPose() const;

    double LeftVelocityReference();

    double RightVelocityReference();

    /**
     * Executes the control loop for a cycle.
     */
    void Update();

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    // The current sensor measurements.
    Eigen::Matrix<double, 2, 1> m_Y;
    double m_headingMeasurement;

    // The control loop.
    frc::StateSpaceLoop<2, 2, 2> m_loop{MakeDrivetrainLoop()};

    // The motion profiles.
    TrapezoidalMotionProfile::Constraints positionConstraints{kRobotMaxV,
                                                              kRobotMaxA};
    TrapezoidalMotionProfile m_positionProfile{positionConstraints,
                                               {0_m, 0_mps}};
    TrapezoidalMotionProfile::Constraints angleConstraints{
        kRobotMaxRotateRate, kRobotMaxRotateAccel};
    TrapezoidalMotionProfile m_angleProfile{angleConstraints, {0_m, 0_mps}};

    Pathfinder m_pathfinder;

    // TrajectoryPoints that store x, y, theta, v, and omega (w) for usage
    TrajectoryPoint m_goal;
    TrajectoryPoint m_estimatedTrajectory;

    bool m_atReferences = false;

    /**
     * Sets the references
     *
     * @param leftVelocity Velocity of the left side in meters per second.
     * @param rightVelocity Velocity of the right side in meters per second.
     */
    void SetReferences(double leftVelocity, double rightVelocity);

    /**
     * Returns sinc(x) = std::sin(x) / x.
     *
     * @param x
     */
    static double Sinc(double x);

    /**
     * Returns a velocity and angular velocity corrected to account for failures
     * to stay on course.
     *
     * @param pose_desired  Desired x, y, and theta values.
     * @param v_desired     Desired velocity.
     * @param omega_desired Desired angular velocity.
     * @param pose          Current x, y, and theta values.
     * @param b             Tuning parameter; makes convergence more aggressive.
     * @param zeta          Tuning parameter in range (0,1); provides damping.
     */
    std::tuple<double, double> Ramsete(TrajectoryPoint trajectory_desired,
                                       TrajectoryPoint trajectory, double b,
                                       double zeta);

    /**
     * Returns left and right velocities from a central velocity and turning
     * rate
     *
     * @param v     Center velocity
     * @param omega Center turning rate
     * @param d     Trackwidth
     */
    std::tuple<double, double> GetDiffVelocities(double v, double omega,
                                                 double d);
};

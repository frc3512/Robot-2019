// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "logging/CsvLogger.hpp"

namespace frc3512 {

class DrivetrainController {
public:
    /**
     * Constructs a drivetrain controller with the given coefficients.
     *
     * @param Qelems The maximum desired error tolerance for each state.
     * @param Relems The maximum desired control effort for each input.
     * @param dt     Discretization timestep.
     */
    DrivetrainController(const std::array<double, 5>& Qelems,
                         const std::array<double, 2>& Relems,
                         units::second_t dt);

    DrivetrainController(const DrivetrainController&) = delete;
    DrivetrainController& operator=(const DrivetrainController&) = delete;

    void Enable();
    void Disable();
    bool IsEnabled() const;

    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal();

    /**
     * Set local measurements.
     *
     * @param heading       Angle of the robot.
     * @param leftVelocity  Velocity of left side in meters per second.
     * @param rightVelocity Velocity of right side in meters per second.
     */
    void SetMeasuredLocalOutputs(units::radian_t heading,
                                 units::meters_per_second_t leftVelocity,
                                 units::meters_per_second_t rightVelocity);

    /**
     * Set global measurements.
     *
     * @param x             X position of the robot in meters.
     * @param y             Y position of the robot in meters.
     * @param heading       Angle of the robot.
     * @param leftVelocity  Velocity of left side in meters per second.
     * @param rightVelocity Velocity of right side in meters per second.
     */
    void SetMeasuredGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::radian_t heading,
                                  units::meters_per_second_t leftVelocity,
                                  units::meters_per_second_t rightVelocity);

    /**
     * Returns the estimated outputs based on the current state estimate.
     *
     * This provides only local measurements.
     */
    Eigen::Matrix<double, 3, 1> EstimatedLocalOutputs() const;

    /**
     * Returns the estimated outputs based on the current state estimate.
     *
     * This provides global measurements (including pose).
     */
    Eigen::Matrix<double, 5, 1> EstimatedGlobalOutputs() const;

    /**
     * Returns the control loop calculated voltage for the left side.
     */
    units::volt_t ControllerLeftVoltage() const;

    /**
     * Returns the control loop calculated voltage for the left side.
     */
    units::volt_t ControllerRightVoltage() const;

    /**
     * Returns the estimated left velocity.
     */
    units::meters_per_second_t EstimatedLeftVelocity() const;

    /**
     * Returns the estimated right velocity.
     */
    units::meters_per_second_t EstimatedRightVelocity() const;

    /**
     * Returns the error between the left velocity reference and the left
     * velocity estimate.
     */
    units::meters_per_second_t LeftVelocityError() const;

    /**
     * Returns the error between the right velocity reference and the right
     * velocity estimate.
     */
    units::meters_per_second_t RightVelocityError() const;

    frc::Pose2d EstimatedPose() const;

    units::meters_per_second_t LeftVelocityReference() const;

    units::meters_per_second_t RightVelocityReference() const;

    /**
     * Executes the control loop for a cycle.
     *
     * @param dt Timestep between each Update() call
     */
    void Update(units::second_t dt, units::second_t elaspedTime);

    /**
     * Resets any internal state.
     */
    void Reset();

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const frc::Pose2d& initialPose);

private:
    // The current sensor measurements.
    Eigen::Matrix<double, 3, 1> m_y;

    units::meter_t m_leftPos = 0_m;
    units::meter_t m_rightPos = 0_m;

    // Design observer
    frc::ExtendedKalmanFilter<5, 2, 3> m_observer{
        Dynamics,
        LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5},
        {0.0001, 0.01, 0.01},
        Constants::kDt};

    // Design controller
    Eigen::Matrix<double, 5, 2> m_B;
    Eigen::Matrix<double, 2, 5> m_K0;
    Eigen::Matrix<double, 2, 5> m_K1;

    // Controller reference
    Eigen::Matrix<double, 5, 1> m_r;

    Eigen::Matrix<double, 5, 1> m_nextR;
    Eigen::Matrix<double, 2, 1> m_cappedU;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;

    wpi::mutex m_trajectoryMutex;

    bool m_atReferences = false;
    bool m_isEnabled = false;

    // The loggers that generates the comma separated value files
    CsvLogger velocityLogger{"DriveVelocities.csv",
                             "Time,LeftRate,RightRate,EstLeftVel,EstRightVel,"
                             "LeftVelRef,RightVelRef"};
    CsvLogger voltageLogger{"DriveVoltages.csv",
                            "Time,LeftVolt,RightVolt,DSVoltage"};
    CsvLogger positionLogger{"DrivePositions.csv",
                             "Time,EstX,EstY,EstTheta,GoalX,GoalY,GoalTheta"};
    CsvLogger errorCovLogger{
        "DriveErrorCov.csv",
        "Time,X Cov,Y Cov,Heading Cov,Left Vel Cov,Right Vel Cov"};

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 5, 1>& x,
        const Eigen::Matrix<double, 5, 1>& r);
    static Eigen::Matrix<double, 5, 1> Dynamics(
        const Eigen::Matrix<double, 5, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);
    static Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 5, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);
    static Eigen::Matrix<double, 5, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 5, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);
};
}  // namespace frc3512

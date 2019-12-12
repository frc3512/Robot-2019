// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/system/NumericalJacobian.h>
#include <frc/system/plant/DCMotor.h>
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
     * Sets the current encoder measurements.
     *
     * @param leftVelocity  Velocity of left side in meters.
     * @param rightVelocity Velocity of right side in meters.
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

    frc::Pose2d EstimatedPose() const;

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
    Eigen::Matrix<double, 3, 1> m_y;

    double m_leftPos = 0.0;
    double m_rightPos = 0.0;

    // Design observers
    frc::UnscentedKalmanFilter<5, 2, 3> m_localObserver{
        Dynamics, LocalMeasurementModel,
        std::array<double, 5>{0.5, 0.5, 10.0, 1.0, 1.0},
        std::array<double, 3>{0.0001, 0.01, 0.01}};
    frc::UnscentedKalmanFilter<5, 2, 5> m_globalObserver{
        Dynamics, GlobalMeasurementModel,
        std::array<double, 5>{0.5, 0.5, 10.0, 1.0, 1.0},
        std::array<double, 5>{0.5, 0.5, 0.0001, 0.01, 0.01}};

    // Design controller
    Eigen::Matrix<double, 2, 5> m_K0;
    Eigen::Matrix<double, 2, 5> m_K1;

    // Controller reference
    Eigen::Matrix<double, 5, 1> m_r;

    Eigen::Matrix<double, 5, 1> m_nextR;
    Eigen::Matrix<double, 2, 1> m_cappedU;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;
    std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::now();

    wpi::mutex m_trajectoryMutex;

    bool m_atReferences = false;
    bool m_isEnabled = false;
    double m_filteredVoltage = 12.0;

    // The loggers that generates the comma separated value files
    CsvLogger velocityLogger{"DriveVelocities.csv",
                             "Time,LeftRate,RightRate,EstLeftVel,EstRightVel,"
                             "GoalV,GoalW,LeftVelRef,RightVelRef"};
    CsvLogger voltageLogger{"DriveVoltages.csv",
                            "Time,LeftVolt,RightVolt,DSVoltage"};
    CsvLogger positionLogger{
        "DrivePositions.csv",
        "Time,LeftPos,RightPos,EstX,EstY,EstTheta,GoalX,GoalY,GoalTheta"};

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

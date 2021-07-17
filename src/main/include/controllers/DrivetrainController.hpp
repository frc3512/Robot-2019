// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <frc/controller/ControlAffinePlantInversionFeedforward.h>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/math>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class DrivetrainController : public ControllerBase<10, 2, 3> {
public:
    class State {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftVelocity = 3;
        static constexpr int kRightVelocity = 4;
        static constexpr int kLeftPosition = 5;
        static constexpr int kRightPosition = 6;
        static constexpr int kLeftVoltageError = 7;
        static constexpr int kRightVoltageError = 8;
        static constexpr int kAngularVelocityError = 9;
    };

    class Input {
    public:
        static constexpr int kLeftVoltage = 0;
        static constexpr int kRightVoltage = 1;
    };

    class LocalOutput {
    public:
        static constexpr int kHeading = 0;
        static constexpr int kLeftPosition = 1;
        static constexpr int kRightPosition = 2;
    };

    class GlobalOutput {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftPosition = 3;
        static constexpr int kRightPosition = 4;
        static constexpr int kAngularVelocity = 5;
    };

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

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    void AddTrajectory(
        const frc::Pose2d& start,
        const std::vector<frc::Translation2d>& interior, const frc::Pose2d& end,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void AddTrajectory(
        const std::vector<frc::Pose2d>& waypoints,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

    bool HaveTrajectory() const;

    void AbortTrajectories();

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Set local measurements.
     *
     * @param heading       Angle of the robot.
     * @param leftPosition  Encoder count of left side in meters.
     * @param rightPosition Encoder count of right side in meters.
     */
    void SetMeasuredLocalOutputs(units::radian_t heading,
                                 units::meter_t leftPosition,
                                 units::meter_t rightPosition);

    /**
     * Set global measurements.
     *
     * @param x             X position of the robot in meters.
     * @param y             Y position of the robot in meters.
     * @param heading       Angle of the robot.
     * @param leftPosition  Encoder count of left side in meters.
     * @param rightPosition Encoder count of right side in meters.
     * @param angularVelocity Angular velocity of the robot in radians per
     * second.
     */
    void SetMeasuredGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::radian_t heading,
                                  units::meter_t leftPosition,
                                  units::meter_t rightPosition,
                                  units::radians_per_second_t angularVelocity);
                                  
    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const frc::Pose2d& initialPose);

    Eigen::Matrix<double, 2, 1> Calculate(
        const Eigen::Matrix<double, 10, 1>& x) override;

    /**
     * Returns the drivetrain's plant.
     */
    static frc::LinearSystem<2, 2, 2> GetPlant();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory config.
     * @param endVelocity The end velocity of the trajectory config.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig(
        units::meters_per_second_t startVelocity,
        units::meters_per_second_t endVelocity);

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 10, 1>& r);

    static Eigen::Matrix<double, 10, 1> Dynamics(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 6, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

private:
    // Robot radius
    static constexpr auto rb = Constants::Drivetrain::kWidth / 2.0;

    static frc::LinearSystem<2, 2, 2> m_plant;

    frc::ControlAffinePlantInversionFeedforward<10, 2> m_ff{
        Dynamics, RealTimeRobot::kDefaultControllerPeriod};

    // Design controller
    // States: [x position, y position, heading, left velocity, right velocity]
    Eigen::Matrix<double, 5, 2> m_B;
    Eigen::Matrix<double, 2, 5> m_K0;
    Eigen::Matrix<double, 2, 5> m_K1;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;
    frc2::Timer m_trajectoryTimeElapsed;

    bool m_atReferences = false;

    /**
     * Constrains theta to within the range (-pi, pi].
     *
     * @param theta Angle to normalize
     */
    static constexpr double NormalizeAngle(double theta) {
        // Constrain theta to within (-3pi, pi)
        const int n_pi_pos = (theta + wpi::math::pi) / 2.0 / wpi::math::pi;
        theta -= n_pi_pos * 2.0 * wpi::math::pi;

        // Cut off the bottom half of the above range to constrain within
        // (-pi, pi]
        const int n_pi_neg = (theta - wpi::math::pi) / 2.0 / wpi::math::pi;
        theta -= n_pi_neg * 2.0 * wpi::math::pi;

        return theta;
    }

    /**
     * Converts velocity and curvature of drivetrain into left and right wheel
     * velocities.
     *
     * @param velocity Linear velocity of drivetrain chassis.
     * @param curvature Curvature of drivetrain arc.
     * @param trackWidth Track width of drivetrain.
     */
    static constexpr std::tuple<units::meters_per_second_t,
                                units::meters_per_second_t>
    ToWheelVelocities(units::meters_per_second_t velocity,
                      units::curvature_t curvature, units::meter_t trackWidth) {
        // clang-format off
        // v = (v_r + v_l) / 2     (1)
        // w = (v_r - v_l) / (2r)  (2)
        // k = w / v               (3)
        //
        // v_l = v - wr
        // v_l = v - (vk)r
        // v_l = v(1 - kr)
        //
        // v_r = v + wr
        // v_r = v + (vk)r
        // v_r = v(1 + kr)
        // clang-format on
        auto vl = velocity * (1 - (curvature / 1_rad * trackWidth / 2.0));
        auto vr = velocity * (1 + (curvature / 1_rad * trackWidth / 2.0));
        return {vl, vr};
    }

    void UpdateAtReferences(const Eigen::Matrix<double, 5, 1>& error);
};
}  // namespace frc3512

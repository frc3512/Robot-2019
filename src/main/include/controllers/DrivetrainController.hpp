// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>
#include <wpi/mutex.h>

#include "Constants.hpp"

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
     * @param leftPosition    Encoder count of left side in meters.
     * @param rightPosition   Encoder count of right side in meters.
     * @param angularVelocity Angular velocity of the robot in radians per
     *                        second.
     */
    void SetMeasuredLocalOutputs(units::meter_t leftPosition,
                                 units::meter_t rightPosition,
                                 units::radians_per_second_t angularVelocity);

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
     * Returns the current references.
     *
     * x, y, heading, left velocity, and right velocity.
     */
    const Eigen::Matrix<double, 5, 1>& GetReferences() const;

    /**
     * Returns the current state estimate.
     *
     * x, y, heading, left position, left velocity, right position,
     * right velocity, left voltage error, right voltage error, and angle error.
     */
    const Eigen::Matrix<double, 10, 1>& GetStates() const;

    /**
     * Returns the control inputs.
     *
     * left voltage and right voltage.
     */
    Eigen::Matrix<double, 2, 1> GetInputs() const;

    /**
     * Returns the currently set local outputs.
     *
     * heading, left position, left velocity, right position,
     * right velocity, and angular velocity.
     */
    const Eigen::Matrix<double, 3, 1>& GetOutputs() const;

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
    Eigen::Matrix<double, 6, 1> EstimatedGlobalOutputs() const;

    /**
     * Executes the control loop for a cycle.
     *
     * @param dt Timestep between each Update() call
     */
    void Update(units::second_t dt, units::second_t elapsedTime);

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

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 5, 1>& r);

    static Eigen::Matrix<double, 10, 1> Dynamics(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 6, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

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
        static constexpr int kLeftPosition = 0;
        static constexpr int kRightPosition = 1;
        static constexpr int kAngularVelocity = 2;
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

private:
    static constexpr auto rb = 0.8382_m / 2.0;  // Robot radius

    static frc::LinearSystem<2, 2, 2> m_plant;

    // The current sensor measurements.
    Eigen::Matrix<double, 3, 1> m_localY;
    Eigen::Matrix<double, 6, 1> m_globalY;

    // Design observer
    // States: [x position, y position, heading,
    //          left velocity, right velocity,
    //          left position, right position,
    //          left voltage error, right voltage error, angle error]
    //
    // Inputs: [left voltage, right voltage]
    //
    // Outputs (local): [left position, right position,
    //                   angular velocity]
    //
    // Outputs (global): [x position, y position, heading,
    //                    left position, right position,
    //                    angular velocity]
    frc::ExtendedKalmanFilter<10, 2, 3> m_observer{
        Dynamics,
        LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5, 10.0, 10.0, 2.0},
        {0.005, 0.005, 0.00001},
        Constants::kDt};

    // Design controller
    // States: [x position, y position, heading, left velocity, right velocity]
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
    frc::CSVLogFile positionLogger{"Drivetrain Positions",
                                   "Estimated X (m)",
                                   "Estimated Y (m)",
                                   "X Ref (m)",
                                   "Y Ref (m)",
                                   "Left Position (m)",
                                   "Right Position (m)",
                                   "Estimated Left Position (m)",
                                   "Estimated Right Position (m)"};
    frc::CSVLogFile angleLogger{"Drivetrain Angles", "Estimated Heading (rad)",
                                "Heading Ref (rad)", "Angle Error (rad)"};
    frc::CSVLogFile velocityLogger{
        "Drivetrain Velocities",     "Left Velocity (m/s)",
        "Right Velocity (m/s)",      "Estimated Left Vel (m/s)",
        "Estimated Right Vel (m/s)", "Left Vel Ref (m/s)",
        "Right Vel Ref (m/s)"};
    frc::CSVLogFile voltageLogger{
        "Drivetrain Voltages",     "Left Voltage (V)",
        "Right Voltage (V)",       "Left Voltage Error (V)",
        "Right Voltage Error (V)", "Battery Voltage (V)"};
    frc::CSVLogFile errorCovLogger{
        "Drivetrain Error Covariances",
        "X Cov (m^2)",
        "Y Cov (m^2)",
        "Heading Cov (rad^2)",
        "Left Vel Cov ((m/s)^2)",
        "Right Vel Cov ((m/s)^2)",
        "Left Pos Cov (m^2)",
        "Right Pos Cov (m^2)",
        "Left Voltage Error Cov (V^2)",
        "Right Voltage Error Cov (V^2)",
        "Angle Error Cov (rad^2)",
    };

    static void ScaleCapU(Eigen::Matrix<double, 2, 1>* u);
};
}  // namespace frc3512

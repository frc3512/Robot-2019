// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/ADXRS450_GyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public ControlledSubsystemBase<10, 2, 3> {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Shifts gear ratio up.
     */
    void ShiftUp();

    /**
     * Shifts gear ratio down.
     */
    void ShiftDown();

    /*
     * Toggles the gear ratio.
     */
    void Shift();

    /**
     * Returns gyro angle.
     */
    units::radian_t GetAngle() const;

    /**
     * Returns gyro angular rate.
     */
    units::radians_per_second_t GetAngularRate() const;

    /**
     * Calibrates gyro.
     */
    void CalibrateGyro();

    /**
     * Returns left encoder displacement.
     */
    units::meter_t GetLeftPosition() const;

    /**
     * Returns right encoder displacement.
     */
    units::meter_t GetRightPosition() const;

    units::meters_per_second_t GetLeftRate() const;

    units::meters_per_second_t GetRightRate() const;

    void Reset(frc::Pose2d initialPose);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end);

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
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end,
                       const frc::TrajectoryConfig& config);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     */
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints);

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
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                       const frc::TrajectoryConfig& config);

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory.
     * @param endVelocity   The end velocity of the trajectory.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig(
        units::meters_per_second_t startVelocity,
        units::meters_per_second_t endVelocity);

    /**
     * Returns whether the drivetrain is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns the current state estimate.
     *
     * x, y, heading, left position, left velocity, right position,
     * right velocity, left voltage error, right voltage error, and angle error.
     */
    const Eigen::Matrix<double, 10, 1>& GetStates() const;

    void DisabledInit() override { Disable(); };

    void AutonomousInit() override { Enable(); };

    void TeleopInit() override;

    void ControllerPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

private:
    frc::Spark m_leftGrbx{Constants::Drivetrain::kLeftMasterPort};
    frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                               Constants::Drivetrain::kLeftEncoderB, false,
                               frc::Encoder::EncodingType::k1X};

    frc::Spark m_rightGrbx{Constants::Drivetrain::kRightMasterPort};
    frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                                Constants::Drivetrain::kRightEncoderB, false,
                                frc::Encoder::EncodingType::k1X};

    frc::ADXRS450_Gyro m_gyro;

    frc::Solenoid m_shifter{Constants::Drivetrain::kShifterPort};

    // XXX: For testing only. This is used to verify the EKF pose because
    // DifferentialDriveOdometry is known to work on other robots.
    frc::DifferentialDriveOdometry m_odometer{frc::Rotation2d(0_rad)};

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
        DrivetrainController::Dynamics,
        DrivetrainController::LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5, 10.0, 10.0, 2.0},
        {0.0001, 0.005, 0.005},
        RealTimeRobot::kDefaultControllerPeriod};

    DrivetrainController m_controller{{0.0625, 0.125, 10.0, 0.95, 0.95},
                                      {12.0, 12.0},
                                      RealTimeRobot::kDefaultControllerPeriod};
    Eigen::Matrix<double, 2, 1> m_u = Eigen::Matrix<double, 2, 1>::Zero();

    // Simulation variables
    // TODO make sure drivetrain is using cim motors
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
        DrivetrainController::GetPlant(), Constants::Drivetrain::kWidth,
        frc::DCMotor::CIM(2), Constants::Drivetrain::kDriveGearRatio,
        Constants::Drivetrain::kWheelRadius};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::sim::ADXRS450_GyroSim m_gyroSim{m_gyro};
    frc::Field2d m_field;
};

}  // namespace frc3512

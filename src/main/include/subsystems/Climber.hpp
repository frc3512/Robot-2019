// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <mutex>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc2/Timer.h>

#include "Constants.hpp"
#include "controllers/ClimberController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

class Climber : public ControlledSubsystemBase<2, 1, 1> {
public:
    /**
     * Constructs a Climber.
     */
    Climber();

    /**
     * Sets the voltage to pass into the drive motor.
     *
     * @param voltage Voltage on [-1..1]
     */
    void SetDriveVoltage(units::volt_t voltage);

    /**
     * Sets the voltage to pass into the lift motor.
     *
     * @param voltage Voltage on [-1..1]
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Resets the encoder.
     */
    void ResetEncoder();

    /**
     * Returns the height of the elevator in meters.
     */
    units::meter_t GetHeight();

    /**
     * Returns the velocity of the elevator in meters per second.
     */
    units::meters_per_second_t GetVelocity();

    /**
     * Sets the goal of the controller.
     *
     * @param position The goal to pass to the controller in meters.
     */
    void SetGoal(units::meter_t position);

    /**
     * Returns whether or not the constorler is at goal.
     */
    bool AtGoal() const;

    void ControllerPeriodic() override;

    void DisabledInit() override { Disable(); };

    void AutonomousInit() override;

    void TeleopInit() override;

    /**
     * Resets sensors and the controller.
     */
    void Reset();

private:
    frc::Spark m_lift{Constants::Climber::kLiftPort};
    frc::Spark m_drive{Constants::Climber::kDrivePort};

    frc::Encoder m_encoder{Constants::Climber::kLiftEncoderA,
                           Constants::Climber::kLiftEncoderB};

    frc::LinearSystem<2, 1, 1> m_plant{ClimberController::GetPlant()};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant,
        {0.05, 1.0},
        {0.0001},
        RealTimeRobot::kDefaultControllerPeriod};
    ClimberController m_controller;
    Eigen::Matrix<double, 1, 1> m_u = Eigen::Matrix<double, 1, 1>::Zero();

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_climberSim{m_controller.GetPlant(),
                                                    {0.0001}};

    frc::sim::EncoderSim m_encoderSim{m_encoder};
};

}  // namespace frc3512

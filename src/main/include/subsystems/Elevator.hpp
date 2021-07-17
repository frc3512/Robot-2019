// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc2/Timer.h>

#include "Constants.hpp"
#include "controllers/ElevatorController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

class Elevator : public ControlledSubsystemBase<2, 1, 1> {
public:
    Elevator();
    Elevator& operator=(const Elevator&) = delete;

    /**
     * Sets the velocity of the elevator.
     *
     * @param velocity in [-12..12]
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Returns height of the elevator.
     *
     * @return height in inches
     */
    units::inch_t GetHeight();

    /**
     * Changes the controller's profile constants to be faster.
     */
    void SetScoringIndex();

    /**
     * Changes the controller's profile constants to be slower.
     */
    void SetClimbingIndex();

    /**
     * Sets the goal of the controller.
     *
     * @param position The goal to pass to the controller in meters.
     */
    void SetGoal(double position);

    /**
     * Returns whether or not the controller is at its references..
     */
    bool AtReference() const;

    /**
     * Returns whether or not the controller has reached its goal.
     */
    bool AtGoal();

    /**
     * Returns the voltage from the controller.
     */
    double ControllerVoltage() const;

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    void DisabledInit() override { Disable(); };

    void AutonomousInit() override { Enable(); };

    void TeleopInit() override { Enable(); };

    void ControllerPeriodic() override;

    void TeleopPeriodic() override;

private:
    rev::CANSparkMax m_grbx{9, rev::CANSparkMax::MotorType::kBrushless};
    frc::Encoder m_encoder{Constants::Elevator::kEncoderA,
                           Constants::Elevator::kEncoderB};

    ElevatorController m_controller;
    frc::LinearSystem<2, 1, 1> m_scorePlant = m_controller.GetScorePlant();
    frc::LinearSystem<2, 1, 1> m_climbPlant = m_controller.GetClimbPlant();
    frc::KalmanFilter<2, 1, 1> m_scoreObserver{
        m_scorePlant,
        {0.05, 100.0},
        {0.0001},
        RealTimeRobot::kDefaultControllerPeriod};
    frc::KalmanFilter<2, 1, 1> m_climbObserver{
        m_climbPlant,
        {0.05, 100.0},
        {0.0001},
        RealTimeRobot::kDefaultControllerPeriod};

    Eigen::Matrix<double, 1, 1> m_u = Eigen::Matrix<double, 1, 1>::Zero();

    // Simulation Variables
    frc::sim::ElevatorSim m_elevatorScoringSim{
        m_controller.GetScorePlant(),
        frc::DCMotor::NEO(),
        Constants::Elevator::kScoringGearRatio,
        units::meter_t{Constants::Elevator::kDrumRadius},
        units::meter_t{Constants::Elevator::kMin},
        units::meter_t{Constants::Elevator::kTopCargo},
        {0.0001}};
    frc::sim::ElevatorSim m_elevatorClimbingSim{
        m_controller.GetClimbPlant(),
        frc::DCMotor::NEO(),
        Constants::Elevator::kClimbingGearRatio,
        units::meter_t{Constants::Elevator::kDrumRadius},
        units::meter_t{Constants::Elevator::kMin},
        units::meter_t{Constants::Elevator::kTopCargo},
        {0.0001}};
    frc::sim::EncoderSim m_encoderSim{m_encoder};
};

}  // namespace frc3512

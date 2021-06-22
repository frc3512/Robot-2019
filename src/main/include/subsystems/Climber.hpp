// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <mutex>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

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
    void SetDriveVoltage(double voltage);

    /**
     * Sets the voltage to pass into the lift motor.
     *
     * @param voltage Voltage on [-1..1]
     */
    void SetVoltage(double voltage);

    /**
     * Resets the encoder.
     */
    void ResetEncoder();

    /**
     * Returns the height of the elevator in meters.
     */
    double GetHeight();

    /**
     * Returns the velocity of the elevator in meters per second.
     */
    double GetVelocity();

    /**
     * Sets the goal of the controller.
     *
     * @param position The goal to pass to the controller in meters.
     */
    void SetGoal(double position);

    /**
     * Returns whether or not the controller is at its references.
     */
    bool AtReference() const;

    /**
     * Returns whether or not the constorler is at goal.
     */
    bool AtGoal() const;

    void ControllerPeriodic() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    /**
     * Returns the voltage from the controller.
     */
    double ControllerVoltage();

    /**
     * Resets sensors and the controller.
     */
    void Reset();

private:
    frc::Spark m_lift{Constants::Climber::kLiftPort};
    frc::Spark m_drive{Constants::Climber::kDrivePort};

    frc::Timer m_timer;

    ClimberController m_controller;

    frc::Encoder m_encoder{Constants::Climber::kLiftEncoderA,
                           Constants::Climber::kLiftEncoderB};
};

}  // namespace frc3512

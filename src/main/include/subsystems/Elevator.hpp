// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Timer.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "controllers/ElevatorController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Elevator : public SubsystemBase, public PublishNode {
public:
    Elevator();
    Elevator& operator=(const Elevator&) = delete;

    /**
     * Sets the velocity of the elevator.
     *
     * @param velocity in [-1..1]
     */
    void SetVoltage(double voltage);

    /**
     * Resets the encoder.
     */
    void ResetEncoder();

    /**
     * Returns height of the elevator.
     *
     * @return height in inches
     */
    double GetHeight();

    /**
     * Runs the control loop every 0.005 seconds.
     */
    void Enable();

    /**
     * Disables the notifier running the control loop.
     */
    void Disable();

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
     * Updates the controller from sensors and the motors from the controller.
     */
    void Iterate();

    /**
     * Returns the voltage from the controller.
     */
    double ControllerVoltage() const;

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    /**
     * Publishes status packets.
     */
    void SubsystemPeriodic() override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

    void ProcessMessage(const HIDPacket& message) override;

private:
    frc::Timer m_timer;
    rev::CANSparkMax m_grbx{9, rev::CANSparkMax::MotorType::kBrushless};

    ElevatorController m_controller;
    frc::Encoder m_encoder{Constants::Elevator::kEncoderA,
                           Constants::Elevator::kEncoderB};

    frc::Notifier m_thread{Constants::kControllerPrio, &Elevator::Iterate,
                           this};

    std::atomic<bool> m_isEnabled{true};
};

}  // namespace frc3512

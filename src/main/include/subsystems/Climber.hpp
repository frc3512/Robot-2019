// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <mutex>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "controllers/ClimberController.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

enum class State {
    kInit,
    kThirdLevel,
    kSecondLevel,
    kFourBarDescend,
    kDescend,
    kDriveForward,
    kIdle
};

class Climber : public SubsystemBase, public PublishNode {
public:
    /**
     * Constructs a Climber.
     *
     * @param pdp The robot's power distribution panel.
     */
    explicit Climber(frc::PowerDistributionPanel& pdp);

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
     * Runs the control loop every 0.005 seconds.
     */
    void Enable();

    /**
     * Disables the notifier running the control loop.
     */
    void Disable();

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
     * Updates the controller from sensors and the motors from the controller.
     */
    void Iterate();

    /**
     * Returns the voltage from the controller.
     */
    double ControllerVoltage();

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    void ProcessMessage(const CommandPacket& message) override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const HIDPacket& message) override;

    void ProcessMessage(const ElevatorStatusPacket& message) override;

    void ProcessMessage(const FourBarLiftStatusPacket& message) override;

    /**
     * Runs a state machine to climb onto platform upon button press
     *
     * The beginning state is the robot on platform level one, up against the
     * level 3 platform. The end state should be the robot on top of the level 3
     * platform with the lift retracted.
     */
    void SubsystemPeriodic() override;

private:
    State m_state = State::kInit;

    frc::Spark m_lift{Constants::Climber::kLiftPort};
    frc::Spark m_drive{Constants::Climber::kDrivePort};

    frc::Timer m_timer;
    double lastVelocity;
    bool m_thirdLevel = true;

    ClimberController m_controller;

    frc::Encoder m_encoder{Constants::Climber::kLiftEncoderA,
                           Constants::Climber::kLiftEncoderB};
    frc::Notifier m_notifier{&Climber::Iterate, this};

    ElevatorStatusPacket m_elevatorStatusPacket;
    FourBarLiftStatusPacket m_fourBarLiftStatusPacket;
    HIDPacket m_HIDPacket;
    ButtonPacket m_buttonPacket;
    std::mutex m_cacheMutex;

    frc::PowerDistributionPanel& m_pdp;
};

}  // namespace frc3512

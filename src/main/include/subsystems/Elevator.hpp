// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "control/ElevatorController.hpp"
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
    // todo: rename to be more accurate
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

    void Enable();
    void Disable();

    void SetScoringIndex();
    void SetClimbingIndex();

    void SetGoal(double position);

    bool AtReference() const;

    bool AtGoal();

    void Iterate();

    double ControllerVoltage() const;

    void Reset();

    void SubsystemPeriodic() override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

private:
    frc::Spark m_grbx{kElevatorPort};

    ElevatorController m_controller;
    frc::Encoder m_encoder{kEncoderA, kEncoderB};

    frc::Notifier m_thread{&Elevator::Iterate, this};

    std::atomic<bool> m_isEnabled{true};
};

}  // namespace frc3512

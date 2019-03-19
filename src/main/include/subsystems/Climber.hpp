// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/PowerDistributionPanel.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

enum class State { kInit, kAscend, kDriveForward, kIdle };

class Climber : public SubsystemBase, public PublishNode {
public:
    void SetLiftVoltage(double voltage);
    void SetDriveVoltage(double voltage);

    /**
     * Pushes the back end of the robot up
     */
    void Ascend();

    /**
     * Allows the back end of the robot down
     */
    void Descend();

    /**
     * Drives the robot forward onto platform
     */
    void Forward();

    /**
     * Drives the robot backward
     */
    void Reverse();

    /**
     * Runs a state machine to climb onto platform
     *
     * The beginning state is the robot on platform level one, up against the
     * level 3 platform. The end state should be the robot on top of the level 3
     * platform with the lift retracted.
     *
     * TODO: Implement the four-bar into the state machine after message queue
     * is finished
     */
    void Climb();

    void ProcessMessage(const HIDPacket& message) override;
    void ProcessMessage(const CommandPacket& message) override;

private:
    State m_state = State::kInit;

    frc::Spark m_lift{kClimberLiftPort};
    frc::Spark m_drive{kClimberDrivePort};
};

}  // namespace frc3512

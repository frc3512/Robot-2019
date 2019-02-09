// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/PowerDistributionPanel.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

#include "Constants.hpp"

enum class State { kInit, kAscend, kDriveForward, kIdle };

/**
 * Provides an interface for this year's climber.
 */
class Climber {
public:
    Climber();

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

private:
    State m_state = State::kInit;

    frc::Timer m_timer;

    frc::Solenoid m_lift{kClimberLiftPort};

    // frc::Spark m_lift{kClimberLiftPort};
    frc::Spark m_drive{kClimberDrivePort};
    frc::PowerDistributionPanel m_pdpDrive{0};
};

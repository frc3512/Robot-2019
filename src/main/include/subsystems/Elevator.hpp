// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>

#include "Constants.hpp"

/**
 * Provides an interface for this year's elevator.
 */
class Elevator {
public:
    Elevator();

    /**
     * Sets the velocity of the elevator.
     *
     * @param velocity in [-1..1]
     */
    // todo: rename to be more accurate
    void SetVelocity(double velocity);

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

private:
    frc::Spark m_master{kElevatorMasterID};
    frc::Spark m_slave{kElevatorSlaveID};
    frc::SpeedControllerGroup m_grbx{m_master, m_slave};

    frc::Encoder m_encoder{kEncoderA, kEncoderB};
};

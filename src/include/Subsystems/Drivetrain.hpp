// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "Constants.hpp"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class Drivetrain {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

private:
    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};
};

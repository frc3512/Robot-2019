// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

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

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Shifts to high gear
    void ShiftUp();

    // Shifts to low gear
    void ShiftDown();

    // Toggles to the other gear
    void Shift();

    // Return gyro's angle
    double GetAngle();

    // Return gyro's rate
    double GetAngularRate() const;

    // Resets gyro
    void ResetGyro();

    // Calibrates gyro
    void CalibrateGyro();

    // Sends print statements for debugging purposes
    void Debug();

    // Returns encoder values
    double GetLeftDisplacement();
    double GetRightDisplacement();

private:
    // Left gearbox used in position PID
    frc::Spark m_leftGrbx{kLeftDriveMasterID};
    frc::Encoder m_leftEncoder{kLeftEncoderA, kLeftEncoderB};

    // Right gearbox used in position PID
    frc::Spark m_rightGrbx{kRightDriveMasterID};
    frc::Encoder m_rightEncoder{kRightEncoderA, kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;

    // Solenoid
    frc::Solenoid m_shifter{kShifterPort};
};

// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public SubsystemBase, public PublishNode {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetLeftManual(double value);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetRightManual(double value);

    /**
     * Shifts gear ratio up.
     */
    void ShiftUp();

    /**
     * Shifts gear ratio down.
     */
    void ShiftDown();

    /*
     * Toggles the gear ratio.
     */
    void Shift();

    /**
     * Returns gyro angle.
     *
     * @return angle in degrees
     */
    double GetAngle();

    /**
     * Returns gyro angular rate.
     *
     * @return angular rate in degrees per second
     */
    double GetAngularRate() const;

    /**
     * Resets gyro.
     */
    void ResetGyro();

    /**
     * Calibrates gyro.
     */
    void CalibrateGyro();

    /**
     * Returns left encoder displacement.
     *
     * @return displacement
     */
    double GetLeftDisplacement();

    /**
     * Returns right encoder displacement.
     *
     * @return displacement
     */
    double GetRightDisplacement();

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

    void ProcessMessage(const HIDPacket& message) override;

private:
    // Left gearbox used in position PID
    frc::Spark m_leftGrbx{kLeftDriveMasterPort};
    frc::Encoder m_leftEncoder{kLeftEncoderA, kLeftEncoderB};

    // Right gearbox used in position PID
    frc::Spark m_rightGrbx{kRightDriveMasterPort};
    frc::Encoder m_rightEncoder{kRightEncoderA, kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;

    // Solenoid
    frc::Solenoid m_shifter{kShifterPort};
};

}  // namespace frc3512

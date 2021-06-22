// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public ControlledSubsystemBase<10, 2, 3> {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

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
     */
    units::radian_t GetAngle() const;

    /**
     * Returns gyro angular rate.
     */
    units::radians_per_second_t GetAngularRate() const;

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
     */
    units::meter_t GetLeftDisplacement() const;

    /**
     * Returns right encoder displacement.
     */
    units::meter_t GetRightDisplacement() const;

    units::meters_per_second_t GetLeftRate() const;

    units::meters_per_second_t GetRightRate() const;

    void ResetEncoders();

    void EnableController();

    void DisableController();

    bool IsControllerEnabled() const;

    void Reset();

    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints);

    bool AtGoal() const;

    void AutonomousInit() override;

    void DisabledInit() override { DisableController(); }

    void ControllerPeriodic() override;

    void TeleopPeriodic() override;

private:
    // Left gearbox used in position PID
    frc::Spark m_leftGrbx{Constants::Drivetrain::kLeftMasterPort};
    frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                               Constants::Drivetrain::kLeftEncoderB, false,
                               frc::Encoder::EncodingType::k1X};

    // Right gearbox used in position PID
    frc::Spark m_rightGrbx{Constants::Drivetrain::kRightMasterPort};
    frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                                Constants::Drivetrain::kRightEncoderB, false,
                                frc::Encoder::EncodingType::k1X};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;

    // Solenoid
    frc::Solenoid m_shifter{Constants::Drivetrain::kShifterPort};

    DrivetrainController m_controller{
        {0.0625, 0.125, 10.0, 0.95, 0.95}, {12.0, 12.0}, Constants::kDt};

    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::time_point::min();

    std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace frc3512

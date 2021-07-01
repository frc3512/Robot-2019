// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/PWMSparkMax.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>

#include "Constants.hpp"
#include "controllers/FourBarLiftController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

class FourBarLift : public ControlledSubsystemBase<2, 1, 1> {
public:
    FourBarLift();
    FourBarLift& operator=(const FourBarLift&) = delete;

    /**
     * Sets the voltage of the elevator.
     *
     * @param voltage in [-1..1]
     */
    void SetVoltage(double voltage);

    /**
     * Sets the controller to climbing mode
     *
     */
    void SetClimbing(bool on);

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
     * Sets the goal of the controller.
     *
     * @param position The goal to pass to the controller in radians.
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
     * Resets sensors and the controller.
     */
    void Reset();

    void DisabledInit() override { Disable(); };

    void AutonomousInit() override { Enable(); };

    void TeleopInit() override { Enable(); };

    void ControllerPeriodic() override;

    void TeleopPeriodic() override;

private:
    rev::CANSparkMax m_grbx{Constants::FourBarLift::kPort,
                            rev::CANSparkMax::MotorType::kBrushless};

    FourBarLiftController m_controller;
    frc::Encoder m_encoder{Constants::FourBarLift::kEncoderA,
                           Constants::FourBarLift::kEncoderB};

    // Simulation variables
    frc::sim::SingleJointedArmSim m_fourBarLiftSim{
        m_controller.GetPlant(),
        frc::DCMotor::NEO(),
        Constants::FourBarLift::kGearRatio,
        units::meter_t{Constants::FourBarLift::kLength},
        units::radian_t{Constants::FourBarLift::kMin},
        units::radian_t{Constants::FourBarLift::kMax},
        units::kilogram_t{Constants::FourBarLift::kMass},
        true,
        {0.01}};
    frc::sim::EncoderSim m_encoderSim{m_encoder};
};

}  // namespace frc3512

// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/PWMSparkMax.h>
#include <frc/SpeedControllerGroup.h>

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

    void ControllerPeriodic() override;

    void TeleopPeriodic() override;

private:
    rev::CANSparkMax m_grbx{Constants::FourBarLift::kPort,
                            rev::CANSparkMax::MotorType::kBrushless};

    FourBarLiftController m_controller;
    frc::Encoder m_encoder{Constants::FourBarLift::kEncoderA,
                           Constants::FourBarLift::kEncoderB};
};

}  // namespace frc3512

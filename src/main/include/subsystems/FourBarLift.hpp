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
     * @param voltage in [-12..12] volts
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Sets the controller to climbing mode
     *
     */
    void SetClimbing(bool on);

    /**
     * Returns height of the elevator.
     *
     * @return height in inches
     */
    units::inch_t GetHeight();

    /**
     * Sets the goal of the controller.
     *
     * @param position The goal to pass to the controller.
     */
    void SetGoal(units::radian_t position);

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
    frc::Encoder m_encoder{Constants::FourBarLift::kEncoderA,
                           Constants::FourBarLift::kEncoderB};

    FourBarLiftController m_controller;
    Eigen::Matrix<double, 1, 1> m_u = Eigen::Matrix<double, 1, 1>::Zero();

    frc::LinearSystem<2, 1, 1> m_plant = m_controller.GetPlant();
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant,
        {0.21745, 0.28726},
        {0.01},
        RealTimeRobot::kDefaultControllerPeriod};

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

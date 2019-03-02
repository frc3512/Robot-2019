// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "control/FourBarLiftController.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class FourBarLift : public SubsystemBase, public PublishNode {
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
     * Resets the encoder.
     */
    void ResetEncoder();

    /**
     * Returns height of the elevator.
     *
     * @return height in inches
     */
    double GetHeight();

    void Enable();
    void Disable();

    void SetGoal(double position);

    bool AtReference() const;

    bool AtGoal();

    void Iterate();

    void Reset();

    void SubsystemPeriodic() override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

private:
    frc::Spark m_grbx{kFourBarLiftPort};

    FourBarLiftController m_controller;
    frc::Encoder m_encoder{kFourBarLiftEncoderA, kFourBarLiftEncoderB};

    frc::Notifier m_thread{&FourBarLift::Iterate, this};
};

}  // namespace frc3512

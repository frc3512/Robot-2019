// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

class Elevator : public SubsystemBase, public PublishNode {
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

    void HallSensor();

    void SubsystemPeriodic() override;

private:
    frc::Spark m_grbx{kElevatorPort};

    frc::Encoder m_encoder{kEncoderA, kEncoderB};

    frc::DigitalInput m_topLimitSwitch{kTopLimitSwitchPort};

    frc::DigitalInput m_bottomLimitSwitch{kBottomLimitSwitchPort};

    bool m_limitPressedState = false;
};

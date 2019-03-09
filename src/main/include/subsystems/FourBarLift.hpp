// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Spark.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class FourBarLift : public SubsystemBase, public PublishNode {
public:
    FourBarLift();

    FourBarLift(const FourBarLift&) = delete;
    FourBarLift& operator=(const FourBarLift&) = delete;

    void SetVoltage(double voltage);

    double GetDisplacement();

    void ResetEncoder();

    bool GetTopLimit();

    void SubsystemPeriodic();

    void ProcessMessage(const HIDPacket& message) override;

private:
    frc::Spark m_motor{kFourBarLiftPort};

    frc::Encoder m_encoder{kFourBarEncoderA, kFourBarEncoderB};
    frc::DigitalInput m_topLimit{kFourBarLimitPort};

    bool m_limitPressedState = true;
};

}  // namespace frc3512

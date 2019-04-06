// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <frc/Spark.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Intake : public SubsystemBase, public PublishNode {
public:
    enum class MotorState { kIntake, kOuttake, kIdle };
    enum class SolenoidState { kOpen, kClose };

    Intake();

    Intake(const Intake&) = delete;
    Intake& operator=(const Intake&) = delete;

    void SetMotors(MotorState motorState);

    void ToggleClaw();

    void ProcessMessage(const ButtonPacket& message);

private:
    frc::Spark m_leftMotor{Constants::Intake::kLeftPort};
    frc::Spark m_rightMotor{Constants::Intake::kRightPort};

    frc::Solenoid m_claw{Constants::Intake::kClawPort};
};

}  // namespace frc3512

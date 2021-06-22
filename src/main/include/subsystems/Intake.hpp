// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <frc/Spark.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Intake : public SubsystemBase {
public:
    enum class MotorState { kIntake, kOuttake, kIdle };
    enum class SolenoidState { kOpen, kClose };

    /**
     * Sets the intake's motors to spin inward or outward.
     */
    void SetMotors(MotorState motorState);

    /**
     * Toggles the claw solenoid.
     */
    void ToggleClaw();

    void TeleopPeriodic() override;

private:
    frc::Spark m_leftMotor{Constants::Intake::kLeftPort};
    frc::Spark m_rightMotor{Constants::Intake::kRightPort};

    frc::Solenoid m_claw{Constants::Intake::kClawPort};
};

}  // namespace frc3512

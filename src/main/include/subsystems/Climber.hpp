// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

#include "Constants.hpp"

enum class State { kInit, kStartClimb, kClimbUp, kIdle };

class Climber {
public:
    Climber();

    // Releases arm onto platfrom
    void DescendArm();
    // Pulls arm back to robot
    void AscendArm();

    // Releases clamps from platform
    void OpenClamps();
    // Presses clamps onto platform
    void CloseClamps();

    // Pulls arm into robot
    void WinchIn();
    // Releases arm onto platform
    void WinchOut();

    // Stops the whinch from recieving voltage
    void WinchStop();

    // Runs the Switch-Case statment that makes the Robot climb
    void Climb();

private:
    State m_state = State::kInit;
    frc::Solenoid m_climberArm{kClimberArmPort};
    frc::DoubleSolenoid m_clamp{kClampForwardPort, kClampReversePort};
    frc::Spark m_winch{kWinchID};
    frc::Timer m_timer;
};

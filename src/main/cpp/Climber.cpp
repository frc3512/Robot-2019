// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

Climber::Climber() { m_timer.Start(); }

void Climber::DescendArm() { m_climberArm.Set(0); }

void Climber::AscendArm() { m_climberArm.Set(1); }

void Climber::CloseClamps() { m_clamp.Set(frc::DoubleSolenoid::kReverse); }

void Climber::OpenClamps() { m_clamp.Set(frc::DoubleSolenoid::kForward); }

void Climber::WinchIn() { m_winch.Set(-1.0); }

void Climber::WinchOut() { m_winch.Set(1.0); }

void Climber::WinchStop() { m_winch.Set(0.0); }

void Climber::Climb() {
    switch (m_state) {
        case State::kInit:
            DescendArm();
            m_state = State::kStartClimb;
            break;
        case State::kStartClimb:
            if (m_timer.HasPeriodPassed(2.5)) {
                CloseClamps();
                m_state = State::kClimbUp;
            }
            break;
        case State::kClimbUp:
            WinchIn();
            // TODO: Needs a condition
            if (1) {
                m_state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}

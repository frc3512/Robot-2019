// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

Climber::Climber() { m_timer.Start(); }

void Climber::Ascend() { m_lift.Set(true); }
// void Climber::Ascend() { m_lift.Set(1.0); }

void Climber::Descend() { m_lift.Set(false); }
// void Climber::Descend() { m_lift.Set(-1.0); }

void Climber::Forward() { m_drive.Set(1.0); }

void Climber::Reverse() { m_drive.Set(-1.0); }

void Climber::Climb() {
    switch (m_state) {
        case State::kInit:
            Ascend();
            m_state = State::kAscend;
            break;
        case State::kAscend:
            if (m_timer.HasPeriodPassed(2.5)) {
                Forward();
                m_state = State::kDriveForward;
            }
        case State::kDriveForward:
            // TODO: Replace HasPeriodPassed with a current check
            if (m_pdpDrive.GetCurrent(kClimberDrivePort) >= 4) {
                Descend();
                m_state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}

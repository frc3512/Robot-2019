// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

using namespace frc3512;

Climber::Climber() { m_timer.Start(); }

void Climber::Ascend() { m_lift.Set(true); }

void Climber::Descend() { m_lift.Set(false); }

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
            if (m_pdpDrive.GetCurrent(kClimberDrivePort) >= 4) {
                Descend();
                m_state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 3 &&
        message.pressed) {
        Ascend();
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 5 &&
        message.pressed) {
        Descend();
    }
}

void Climber::ProcessMessage(const POVPacket& message) {
    if (message.topic == "Robot/AppendagePOV" && message.direction == 0) {
        Forward();
    }
    if (message.topic == "Robot/AppendagePOV" && message.direction == 180) {
        Reverse();
    }
    if (message.topic == "Robot/AppendagePOV" && message.direction == -1) {
        m_drive.Set(0.0);
    }
}

void Climber::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    }
}

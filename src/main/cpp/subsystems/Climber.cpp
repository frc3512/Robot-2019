// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

using namespace frc3512;

void Climber::SetLiftVoltage(double voltage) { m_lift.Set(voltage); }

void Climber::SetDriveVoltage(double voltage) { m_drive.Set(voltage); }

void Climber::Ascend() { m_lift.Set(1.0); }

void Climber::Descend() { m_lift.Set(-1.0); }

void Climber::Forward() { m_drive.Set(1.0); }

void Climber::Reverse() { m_drive.Set(-1.0); }

void Climber::Climb() {
    switch (m_state) {
        case State::kInit:
            Ascend();
            m_state = State::kAscend;
            break;
        case State::kAscend:
            // TODO: Replace with controller
            if (1) {
                Forward();
                m_state = State::kDriveForward;
            }
        case State::kDriveForward:
            if (Robot::pdp.GetCurrent(kClimberDrivePort) >= 4) {
                Descend();
                m_state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}

void Climber::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    }
}

void Climber::ProcessMessage(const POVPacket& message) {
    if (message.topic == "Robot/AppendagePOV" && message.direction == 0) {
        SetDriveVoltage(0.5);
    } else if (message.topic == "Robot/AppendagePOV" &&
               message.direction == 180) {
        SetDriveVoltage(-0.5);
    } else {
        SetDriveVoltage(0);
    }
}

void Climber::ProcessMessage(const HIDPacket& message) {
    SetLiftVoltage(message.y3 * 0.5);
}

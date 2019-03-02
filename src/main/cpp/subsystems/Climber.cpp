// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <chrono>

#include <frc/DriverStation.h>

#include "Robot.hpp"

using namespace frc3512;
using namespace std::chrono_literals;

Climber::Climber() : PublishNode("Climber") {
    m_encoder.SetReverseDirection(true);
    m_encoder.SetDistancePerPulse(kClimberDpP);
    m_timer.Start();
    Subscribe(*this);
}

void Climber::SetLiftVoltage(double voltage) { m_lift.Set(voltage); }

void Climber::Forward() { m_drive.Set(1.0); }

void Climber::SetDriveVoltage(double voltage) { m_drive.Set(voltage); }

void Climber::Backward() { m_drive.Set(-1.0); }

void Climber::Up() { m_lift.Set(1.0); }

void Climber::Down() { m_lift.Set(-1.0); }

void Climber::Stop() { m_drive.Set(0); }

void Climber::SetVoltage(double voltage) {
    /*if(voltage > 0.0 && m_topLimitSwitch.Get() == m_limitPressedState) {
        voltage = 0.0;
    }
    if (voltage < 0.0 && m_bottomLimitSwitch.Get() == m_limitPressedState) {
        voltage = 0.0;
    }
    if (voltage > 0.0 && m_encoder.GetDistance() >= kElevatorMax) {
        voltage = 0.0;
    }*/
    m_lift.Set(voltage);
}

void Climber::ResetEncoder() { m_encoder.Reset(); }

double Climber::GetHeight() { return m_encoder.GetDistance(); }

double Climber::GetVelocity() { return m_encoder.GetRate(); }

void Climber::Enable() {
    m_controller.Enable();
    m_notifier.StartPeriodic(0.005);
}

void Climber::Disable() {
    m_controller.Disable();
    m_notifier.Stop();
}

void Climber::Iterate() {
    m_controller.SetMeasuredPosition(m_encoder.GetDistance());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();

    if (m_controller.ErrorExceeded()) {
        SetVoltage(
            (m_controller.ControllerVoltage() + (m_HIDPacket.y2 * -0.4)) /
            batteryVoltage);
    } else {
        SetVoltage(m_controller.ControllerVoltage() / batteryVoltage);
    }
    /*std::cout << m_controller.PositionReference() << " || "
              << m_controller.EstimatedPosition() << " || "
              << m_controller.ControllerVoltage() << " || "
              << m_controller.EstimatedVelocity() << std::endl;*/
}

void Climber::SetGoal(double position) { m_controller.SetGoal(position); }

bool Climber::AtReference() const { return m_controller.AtReferences(); }

double Climber::ControllerVoltage() { return m_controller.ControllerVoltage(); }

void Climber::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

void Climber::SubsystemPeriodic() {
    switch (m_state) {
        case State::kInit: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            std::cout << "Init" << std::endl;
            if (m_buttonPacket.topic == "Robot/AppendageStick2" &&
                m_buttonPacket.button == 7 && m_buttonPacket.pressed) {
                m_buttonPacket.pressed = false;
                CommandPacket message{"ElevatorStart", true};
                Publish(message);
                m_state = State::kElevatorRaise;
            }
            break;
        }
        case State::kElevatorRaise: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            std::cout << "ElevatorRaise" << std::endl;
            if (m_elevatorStatusPacket.atGoal) {
                CommandPacket message0{"FourBarStart", true};
                Publish(message0);
                m_state = State::kFourBarRaise;
            }
            break;
        }
        case State::kFourBarRaise: {
            std::cout << "FourBarRaise" << std::endl;
            if (m_fourBarLiftStatusPacket.atGoal) {
                CommandPacket message1{"ClimbingProfile", false};
                Publish(message1);
                CommandPacket message2{"Down", false};
                Publish(message2);
                m_state = State::kDescend;
            }
            break;
        }
        case State::kDescend: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            if (m_controller.AtGoal() && m_elevatorStatusPacket.atGoal) {
                m_state = State::kDriveForward;
            }
            break;
        }
        case State::kDriveForward: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            m_drive.Set(m_HIDPacket.y1);
            if (m_buttonPacket.topic == "Robot/AppendageStick2" &&
                m_buttonPacket.button == 7 && m_buttonPacket.pressed) {
                m_buttonPacket.pressed = false;
                CommandPacket message{"Up", false};
                Publish(message);
                m_state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            CommandPacket message{"ScoringProfile", false};
            Publish(message);
            break;
        }
    }
}

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 7 &&
        message.pressed) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        m_buttonPacket = message;
    }
}

void Climber::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit") {
        EnablePeriodic();
        Enable();
        SetGoal(0.035);
    }
    if (message.topic == "Climber/Down") {
        SetGoal(kClimbHeight);
    }
    if (message.topic == "Climber/Up") {
        SetGoal(0);
    }
}

void Climber::ProcessMessage(const HIDPacket& message) {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_HIDPacket = message;
}

void Climber::ProcessMessage(const ElevatorStatusPacket& message) {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_elevatorStatusPacket = message;
}

void Climber::ProcessMessage(const FourBarLiftStatusPacket& message) {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_fourBarLiftStatusPacket = message;
}

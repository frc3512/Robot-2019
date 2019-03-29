// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <chrono>

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

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
    /*wpi::outs() << m_controller.PositionReference() << " || "
              << m_controller.EstimatedPosition() << " || "
              << m_controller.ControllerVoltage() << " || "
              << m_controller.EstimatedVelocity() << "\n";*/
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
            wpi::outs() << "Init\n";
            if (m_buttonPacket.topic == "Robot/AppendageStick2" &&
                m_buttonPacket.button == 7 && m_buttonPacket.pressed) {
                m_buttonPacket.pressed = false;
                m_thirdLevel = true;
                CommandPacket message{"ThirdLevel", true};
                Publish(message);
                m_state = State::kThirdLevel;
            }
            if (m_buttonPacket.topic == "Robot/AppendageStick2" &&
                m_buttonPacket.button == 8 && m_buttonPacket.pressed) {
                m_buttonPacket.pressed = false;
                m_thirdLevel = false;
                CommandPacket message{"SecondLevel", true};
                Publish(message);
                m_state = State::kSecondLevel;
            }
            break;
        }
        case State::kThirdLevel: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            wpi::outs() << "ThirdLevel\n";
            if (m_elevatorStatusPacket.atGoal &&
                m_elevatorStatusPacket.distance > 0.3) {
                CommandPacket message0{"FourBarStart", true};
                Publish(message0);
                m_state = State::kFourBarDescend;
            }
            break;
        }
        case State::kSecondLevel: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            wpi::outs() << "SecondLevel\n";
            if (m_elevatorStatusPacket.atGoal &&
                m_elevatorStatusPacket.distance > 0.1) {
                CommandPacket message0{"FourBarStart", true};
                Publish(message0);
                m_state = State::kFourBarDescend;
            }
            break;
        }
        case State::kFourBarDescend: {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            wpi::outs() << "FourBarDescend\n";
            if (m_fourBarLiftStatusPacket.atGoal &&
                m_fourBarLiftStatusPacket.distance < -1.0) {
                CommandPacket message1{"ClimbingProfile", false};
                Publish(message1);
                if (m_thirdLevel) {
                    CommandPacket message2{"Down3", false};
                    Publish(message2);
                } else {
                    CommandPacket message2{"Down2", false};
                    Publish(message2);
                }
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
            m_drive.Set(-m_HIDPacket.y1);
            if (m_buttonPacket.topic == "Robot/AppendageStick2" &&
                m_buttonPacket.button == 9 && m_buttonPacket.pressed) {
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
    if (message.topic == "Robot/AppendageStick2" && message.button == 8 &&
        message.pressed) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        m_buttonPacket = message;
    }
    if (message.topic == "Robot/AppendageStick2" && message.button == 9 &&
        message.pressed) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        m_buttonPacket = message;
    }
}

void Climber::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/AutonomousInit") {
        EnablePeriodic();
        Enable();
        SetGoal(0.03);
    }
    if (message.topic == "Robot/TeleopInit") {
        EnablePeriodic();
        Enable();
        SetGoal(0.03);
    }
    if (message.topic == "Climber/Down3") {
        SetGoal(kClimb3Height);
    }
    if (message.topic == "Climber/Down2") {
        SetGoal(kClimb2Height);
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

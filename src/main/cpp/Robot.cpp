// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <frc/Joystick.h>
#include <wpi/raw_ostream.h>

namespace frc3512 {

Robot::Robot() {
    camera.SetResolution(160, 120);
    camera.SetFPS(15);
    server.SetSource(camera);

    Schedule(
        [=] {
            if (IsEnabled()) {
                m_drivetrain.ControllerPeriodic();
            }
        },
        1_ms);
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_climber.ControllerPeriodic();
            }
        },
        0.6_ms);
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_elevator.ControllerPeriodic();
            }
        },
        0.6_ms);
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_fourBarLift.ControllerPeriodic();
            }
        },
        0.6_ms);
}

void Robot::DisabledInit() { SubsystemBase::RunAllDisabledInit(); }

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    m_drivetrain.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});
}

void Robot::TeleopInit() { SubsystemBase::RunAllTeleopInit(); }

void Robot::TestInit() { SubsystemBase::RunAllTestInit(); }

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    static frc::Joystick appendageStick1{Constants::Robot::kAppendageStickPort};
    static frc::Joystick appendageStick2{
        Constants::Robot::kAppendageStick2Port};

    switch (m_state) {
        case State::kInit: {
            std::lock_guard lock(m_cacheMutex);
            if (appendageStick2.GetRawButtonPressed(7)) {
                m_elevator.SetGoal(Constants::Elevator::kHab3);
                m_thirdLevel = true;
                m_state = State::kThirdLevel;
            }
            if (appendageStick2.GetRawButtonPressed(8)) {
                m_elevator.SetGoal(Constants::Elevator::kHab2);
                m_thirdLevel = false;
                m_state = State::kSecondLevel;
            }
            break;
        }
        case State::kThirdLevel: {
            std::lock_guard lock(m_cacheMutex);
            wpi::outs() << "ThirdLevel\n";
            if (m_elevator.AtGoal() && m_elevator.GetHeight() > 0.3) {
                // Hardcoded number from who knows where
                m_fourBarLift.SetClimbing(true);
                m_fourBarLift.SetGoal(-1.35);
                m_state = State::kFourBarDescend;
            }
            break;
        }
        case State::kSecondLevel: {
            std::lock_guard lock(m_cacheMutex);
            wpi::outs() << "SecondLevel\n";
            if (m_elevator.AtGoal() && m_elevator.GetHeight() > 0.1) {
                m_fourBarLift.SetClimbing(true);
                m_fourBarLift.SetGoal(-1.35);
                m_state = State::kFourBarDescend;
            }
            break;
        }
        case State::kFourBarDescend: {
            std::lock_guard lock(m_cacheMutex);
            if (m_fourBarLift.AtGoal() && m_fourBarLift.GetHeight() > -0.7) {
                m_elevator.SetClimbingIndex();
                m_elevator.SetGoal(0);
                if (m_thirdLevel) {
                    m_climber.SetGoal(
                        units::meter_t{Constants::Climber::kClimb3Height});
                } else {
                    m_climber.SetGoal(
                        units::meter_t{Constants::Climber::kClimb2Height});
                }
                m_state = State::kDescend;
            }
            break;
        }
        case State::kDescend: {
            std::lock_guard lock(m_cacheMutex);
            if (m_climber.AtGoal() && m_elevator.AtGoal()) {
                m_state = State::kDriveForward;
            }
            break;
        }
        case State::kDriveForward: {
            std::lock_guard lock(m_cacheMutex);
            m_climber.SetDriveVoltage(
                units::volt_t{appendageStick1.GetY() * 12});
            if (appendageStick2.GetRawButtonPressed(9)) {
                m_fourBarLift.SetClimbing(false);
                m_fourBarLift.SetGoal(0);
                m_climber.SetGoal(0_m);
                m_state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            m_elevator.SetScoringIndex();
            break;
        }
    }
}

void Robot::DisabledPeriodic() {
    SubsystemBase::RunAllDisabledPeriodic();

    wpi::outs() << "FourBar: " << m_fourBarLift.GetHeight() << "\n";
    wpi::outs() << "Elevator: " << m_elevator.GetHeight() << "\n";
    wpi::outs() << "Climber: " << m_climber.GetHeight().to<double>() << "\n";
    wpi::outs() << "Drivetrain Left: "
                << static_cast<double>(m_drivetrain.GetLeftDisplacement())
                << "\n";
    wpi::outs() << "Drivetrain Right: "
                << static_cast<double>(m_drivetrain.GetRightDisplacement())
                << "\n";
    wpi::outs() << "Drivetrain Gyro: "
                << static_cast<double>(m_drivetrain.GetAngle()) << "\n";
    wpi::outs().flush();
}

void Robot::AutonomousPeriodic() { SubsystemBase::RunAllAutonomousPeriodic(); }

void Robot::TeleopPeriodic() { SubsystemBase::RunAllTeleopPeriodic(); }

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif

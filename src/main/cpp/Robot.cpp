// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include <string>

#include <cscore.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>
#include <wpi/raw_ostream.h>

#include "Constants.hpp"
#include "logging/LogFileSink.hpp"
#include "logging/Logger.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/FourBarLift.hpp"
#include "subsystems/Intake.hpp"

using namespace frc3512::Constants::Robot;

namespace frc3512 {

class Robot : public frc::TimedRobot, public PublishNode {
public:
    Robot() : PublishNode("Robot") {
        m_logger.AddLogSink(fileSink);
        m_logger.Subscribe(m_climber);
        m_logger.Subscribe(m_drivetrain);
        m_logger.Subscribe(m_elevator);
        m_logger.Subscribe(m_intake);
        m_logger.Subscribe(m_fourBarLift);
        m_logger.Subscribe(*this);

        m_climber.Subscribe(*this);
        m_climber.Subscribe(m_climber);
        m_climber.Subscribe(m_elevator);
        m_climber.Subscribe(m_fourBarLift);
        m_drivetrain.Subscribe(*this);
        m_elevator.Subscribe(*this);
        m_elevator.Subscribe(m_climber);
        m_elevator.Subscribe(m_fourBarLift);
        m_intake.Subscribe(*this);
        m_fourBarLift.Subscribe(*this);
        m_fourBarLift.Subscribe(m_elevator);
        m_fourBarLift.Subscribe(m_climber);

        camera.SetResolution(160, 120);
        camera.SetFPS(15);
        server.SetSource(camera);

        m_fourBarLift.Subscribe(m_climber);

        frc::LiveWindow::GetInstance()->DisableAllTelemetry();
    }

    void DisabledInit() override {
        CommandPacket message{"DisabledInit", false};
        Publish(message);
    }

    void AutonomousInit() override {
        CommandPacket message{"AutonomousInit", false};
        Publish(message);
    }

    void TeleopInit() override {
        CommandPacket message{"TeleopInit", false};
        Publish(message);

        for (int i = 1; i <= 12; i++) {
            if (m_driveStick2.GetRawButtonPressed(i)) {
                ButtonPacket message{"DriveStick2", i, true};
                Publish(message);
            }
            if (m_appendageStick.GetRawButtonPressed(i)) {
                ButtonPacket message{"AppendageStick", i, true};
                Publish(message);
            }
            if (m_appendageStick.GetRawButtonReleased(i)) {
                ButtonPacket message{"AppendageStick", i, false};
                Publish(message);
            }
            if (m_appendageStick2.GetRawButtonPressed(i)) {
                ButtonPacket message{"AppendageStick2", i, true};
                wpi::outs() << "h\n";
                Publish(message);
            }
            if (m_appendageStick2.GetRawButtonReleased(i)) {
                ButtonPacket message{"AppendageStick2", i, false};
                wpi::outs() << "r\n";
                Publish(message);
            }
        }
    }

    void TestInit() override {}

    void RobotPeriodic() override {}

    void DisabledPeriodic() override {
        wpi::outs() << "FourBar: " << m_fourBarLift.GetHeight() << "\n";
        wpi::outs() << "Elevator: " << m_elevator.GetHeight() << "\n";
        wpi::outs() << "Climber: " << m_climber.GetHeight() << "\n";
    }

    void AutonomousPeriodic() override { TeleopPeriodic(); }

    void TeleopPeriodic() override {
        for (int i = 1; i <= 12; i++) {
            if (m_driveStick2.GetRawButtonPressed(i)) {
                ButtonPacket message{"DriveStick2", i, true};
                Publish(message);
            }
            if (m_appendageStick.GetRawButtonPressed(i)) {
                ButtonPacket message{"AppendageStick", i, true};
                Publish(message);
            }
            if (m_appendageStick.GetRawButtonReleased(i)) {
                ButtonPacket message{"AppendageStick", i, false};
                Publish(message);
            }
            if (m_appendageStick2.GetRawButtonPressed(i)) {
                ButtonPacket message{"AppendageStick2", i, true};
                Publish(message);
            }
            if (m_appendageStick2.GetRawButtonReleased(i)) {
                ButtonPacket message{"AppendageStick2", i, false};
                Publish(message);
            }
        }

        auto& ds = frc::DriverStation::GetInstance();
        HIDPacket message{"",
                          m_driveStick1.GetX(),
                          m_driveStick1.GetY(),
                          ds.GetStickButtons(0),
                          m_driveStick2.GetX(),
                          m_driveStick2.GetY(),
                          ds.GetStickButtons(1),
                          m_appendageStick.GetX(),
                          m_appendageStick.GetY(),
                          ds.GetStickButtons(2),
                          m_appendageStick2.GetX(),
                          m_appendageStick2.GetY(),
                          ds.GetStickButtons(3)};
        Publish(message);
    }

private:
    frc::PowerDistributionPanel m_pdp;
    Climber m_climber{m_pdp};
    Drivetrain m_drivetrain;
    Elevator m_elevator;
    Logger m_logger;
    Intake m_intake;
    FourBarLift m_fourBarLift;

    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStickPort};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};

    LogFileSink fileSink{"/home/lvuser/Robot.log"};

    cs::UsbCamera camera{"Camera 1", 0};
    cs::MjpegServer server{"Server", kMjpegServerPort};
};

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif

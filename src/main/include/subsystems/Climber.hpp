// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <mutex>

#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/Spark.h>
#include <frc/Timer.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "control/ClimberController.hpp"
#include "logging/CsvLogger.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

enum class State {
    kInit,
    kThirdLevel,
    kSecondLevel,
    kFourBarDescend,
    kDescend,
    kDriveForward,
    kIdle
};

class Climber : public SubsystemBase, public PublishNode {
public:
    Climber();

    void SetLiftVoltage(double voltage);
    void SetDriveVoltage(double voltage);

    /**
     * Pushes the back end of the robot up
     */
    void Up();

    /**
     * Allows the back end of the robot down
     */
    void Down();

    /**
     * Drives the robot forward onto platform
     */
    void Forward();

    /**
     * Drives the robot backward
     */
    void Backward();

    /**
     * Stops the robot
     */
    void Stop();

    void SetVoltage(double voltage);

    /**
     * Resets the encoder.
     */
    void ResetEncoder();

    /**
     * Returns height of the elevator.
     *
     * @return height in inches
     */
    double GetHeight();

    double GetVelocity();

    void Enable();
    void Disable();

    void SetGoal(double position);

    bool AtReference() const;

    void Iterate();

    double ControllerVoltage();

    void Reset();

    void ProcessMessage(const CommandPacket& message) override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const HIDPacket& message) override;

    void ProcessMessage(const ElevatorStatusPacket& message) override;

    void ProcessMessage(const FourBarLiftStatusPacket& message) override;

    /**
     * Runs a state machine to climb onto platform upon button press
     *
     * The beginning state is the robot on platform level one, up against the
     * level 3 platform. The end state should be the robot on top of the level 3
     * platform with the lift retracted.
     */
    void SubsystemPeriodic() override;

private:
    State m_state = State::kInit;

    frc::Spark m_lift{kClimberLiftPort};
    frc::Spark m_drive{kClimberDrivePort};

    frc::Timer m_timer;
    double lastVelocity;
    bool m_thirdLevel = true;

    ClimberController m_controller;

    frc::PowerDistributionPanel m_pdpDrive{0};
    frc::Encoder m_encoder{kLiftEncoderA, kLiftEncoderB};
    frc::Notifier m_notifier{&Climber::Iterate, this};
    CsvLogger climberLogger{"/home/lvuser/ClimberStuff.csv",
                            "Time,Pos,Velo,Accel"};

    ElevatorStatusPacket m_elevatorStatusPacket;
    FourBarLiftStatusPacket m_fourBarLiftStatusPacket;
    HIDPacket m_HIDPacket;
    ButtonPacket m_buttonPacket;
    std::mutex m_cacheMutex;
};

}  // namespace frc3512

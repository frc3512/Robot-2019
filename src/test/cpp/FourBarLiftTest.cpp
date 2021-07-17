// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/FourBarLift.hpp"

class FourBarLiftTest : public frc3512::SimulatorTest {
public:
    frc3512::FourBarLift fourBarLift;

    frc::Notifier controllerPeriodic{[&] {
        fourBarLift.TeleopPeriodic();
        fourBarLift.ControllerPeriodic();
    }};

    FourBarLiftTest() {
        frc3512::SubsystemBase::RunAllTeleopInit();
        controllerPeriodic.StartPeriodic(
            frc3512::RealTimeRobot::kDefaultControllerPeriod);
    }
};

TEST_F(FourBarLiftTest, ReachesReferenceDown) {
    frc::sim::JoystickSim appendageStick2{
        frc3512::Constants::Robot::kAppendageStick2Port};

    appendageStick2.SetRawButton(3, true);
    appendageStick2.NotifyNewData();
    frc::sim::StepTiming(5_s);

    EXPECT_TRUE(fourBarLift.AtGoal());

    appendageStick2.SetRawButton(2, true);
    appendageStick2.NotifyNewData();
    frc::sim::StepTiming(5_s);

    EXPECT_TRUE(fourBarLift.AtGoal());
}

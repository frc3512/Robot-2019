// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>

#include "Robot.hpp"
#include "SimulatorTest.hpp"

class RobotTest : public frc3512::SimulatorTest {
public:
    frc3512::Robot robot;

    RobotTest() {
        ds.SetAutonomous(false);
        ds.SetEnabled(true);
        ds.NotifyNewData();
    }

    ~RobotTest() override {
        robot.EndCompetition();
        robotThread.join();
    }

private:
    std::thread robotThread{[&] { robot.StartCompetition(); }};
    frc::sim::DriverStationSim ds;
};

// Make sure robot initializes
TEST_F(RobotTest, Init) {}

TEST_F(RobotTest, SecondLevelClimb) {
    frc::sim::JoystickSim appendageStick2{
        frc3512::Constants::Robot::kAppendageStick2Port};

    appendageStick2.SetRawButton(8, true);
    appendageStick2.NotifyNewData();

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(robot.IsClimbing());
}

TEST_F(RobotTest, ThirdLevelClimb) {
    frc::sim::JoystickSim appendageStick2{
        frc3512::Constants::Robot::kAppendageStick2Port};

    appendageStick2.SetRawButton(7, true);
    appendageStick2.NotifyNewData();

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(robot.IsClimbing());
}

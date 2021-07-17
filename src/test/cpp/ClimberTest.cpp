// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"

using namespace frc3512::Constants::Climber;

class ClimberTest : public frc3512::SimulatorTest {
public:
    frc3512::Climber climber;

    frc::Notifier controllerPeriodic{[&] {
        climber.ControllerPeriodic();
    }};

    ClimberTest() {
        controllerPeriodic.StartPeriodic(
            frc3512::RealTimeRobot::kDefaultControllerPeriod);
        climber.Enable();
    }
};

TEST_F(ClimberTest, MoveClimberUp) { 
    climber.SetGoal(units::meter_t{kRaisedUp});

    EXPECT_TRUE(climber.AtGoal());
}

TEST_F(ClimberTest, ClimbHeights) {
    climber.SetGoal(units::meter_t{kClimb2Height});

    EXPECT_TRUE(climber.AtGoal());

    climber.SetGoal(units::meter_t{kClimb3Height});

    EXPECT_TRUE(climber.AtGoal());

    climber.SetGoal(0_m);
    EXPECT_TRUE(climber.AtGoal());
}
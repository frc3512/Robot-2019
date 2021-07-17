// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"

class ClimberTest : public frc3512::SimulatorTest {
public:
    frc3512::Climber climber;
};

TEST_F(ClimberTest, MoveClimberUp) { frc2::Timer timer; }

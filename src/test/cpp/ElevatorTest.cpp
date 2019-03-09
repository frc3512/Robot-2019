// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>

#include "subsystems/Elevator.hpp"

using namespace std::chrono_literals;

class ElevatorTest : public testing::Test {
protected:
    Elevator elevator;
};

TEST_F(ElevatorTest, Elevator) {
    elevator.Reset();
    elevator.SetGoal(5);

    std::this_thread::sleep_for(1000ms);

    EXPECT_TRUE(elevator.AtReference());
}

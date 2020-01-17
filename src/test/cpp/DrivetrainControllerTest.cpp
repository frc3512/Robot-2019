// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/StateSpaceUtil.h>
#include <gtest/gtest.h>
#include <units/units.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"

TEST(DrivetrainControllerTest, ReachesReference) {
    using frc3512::Constants::kDt;

    frc3512::DrivetrainController controller{
        {0.0625, 0.125, 10.0, 0.95, 0.95}, {12.0, 12.0}, kDt};
    controller.Reset(frc::Pose2d{0_m, 0_m, 0_rad});
    controller.Enable();

    controller.SetMeasuredLocalOutputs(0_rad, 0_mps, 0_mps);
    controller.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});

    auto currentTime = 0_s;
    while (!controller.AtGoal() && currentTime < 10_s) {
        Eigen::Matrix<double, 3, 1> noise =
            frc::MakeWhiteNoiseVector(0.0001, 0.01, 0.01);
        controller.SetMeasuredLocalOutputs(
            controller.EstimatedPose().Rotation().Radians() +
                units::radian_t{noise(0, 0)},
            controller.EstimatedLeftVelocity() +
                units::meters_per_second_t{noise(1, 0)},
            controller.EstimatedRightVelocity() +
                units::meters_per_second_t{noise(2, 0)});
        controller.Update(kDt, currentTime);
        currentTime += kDt;
    }
    EXPECT_TRUE(controller.AtGoal());
    EXPECT_LT(currentTime, 10_s);
}

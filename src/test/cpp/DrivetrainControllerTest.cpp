// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "controllers/DrivetrainController.hpp"

TEST(DrivetrainControllerTest, ReachesReference) {
    using namespace frc3512::Constants::Drivetrain;
    using namespace frc3512;

    frc3512::DrivetrainController controller{
        {0.0625, 0.125, 10.0, 0.95, 0.95},
        {12.0, 12.0},
        RealTimeRobot::kDefaultControllerPeriod};
    controller.Reset(frc::Pose2d{0_m, 0_m, 0_rad});

    controller.SetMeasuredLocalOutputs(0_rad, 0_m, 0_m);
    controller.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});

    frc::sim::DifferentialDrivetrainSim drivetrainSim{
        controller.GetPlant(),
        kWidth,
        frc::DCMotor::MiniCIM(3),
        kDriveGearRatio,
        kWheelRadius,
        {0.0, 0.0, 0.0001, 0.0, 0.0, 0.005, 0.005}};

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        controller.SetMeasuredLocalOutputs(drivetrainSim.GetHeading().Radians(),
                                           drivetrainSim.GetLeftPosition(),
                                           drivetrainSim.GetRightPosition());
        controller.Update(RealTimeRobot::kDefaultControllerPeriod, currentTime);
        currentTime += RealTimeRobot::kDefaultControllerPeriod;

        Eigen::Matrix<double, 2, 1> u = controller.GetInputs();

        // frc::sim::RoboRioSim::SetVInVoltage(
        //     frc::sim::BatterySim::Calculate({drivetrainSim.GetCurrentDraw()}));

        u *= frc::RobotController::GetInputVoltage() / 12.0;

        drivetrainSim.SetInputs(units::volt_t{u(0)}, units::volt_t{u(1)});
        drivetrainSim.Update(RealTimeRobot::kDefaultControllerPeriod);
    }
    EXPECT_TRUE(controller.AtGoal());
}

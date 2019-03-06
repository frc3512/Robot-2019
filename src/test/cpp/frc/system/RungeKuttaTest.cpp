/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <gtest/gtest.h>

#include <cmath>

#include "frc/system/RungeKutta.h"

// Tests that integrating dx/dt = e^x works.
TEST(RungeKutta, Exponential) {
  Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 0.0;

  Eigen::Matrix<double, 1, 1> y1 = frc::RungeKutta(
      [](Eigen::Matrix<double, 1, 1> x) {
        Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = std::exp(x(0, 0));
        return y;
      },
      y0, 0.1_s);
  EXPECT_NEAR(y1(0, 0), std::exp(0.1) - std::exp(0), 1e-3);
}

// Tests that integrating dx/dt = e^x works when we provide a U.
TEST(RungeKutta, ExponentialWithU) {
  Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 0.0;

  Eigen::Matrix<double, 1, 1> y1 = frc::RungeKutta(
      [](Eigen::Matrix<double, 1, 1> x, Eigen::Matrix<double, 1, 1> u) {
        Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = std::exp(u(0, 0) * x(0, 0));
        return y;
      },
      y0, (Eigen::Matrix<double, 1, 1>() << 1.0).finished(), 0.1_s);
  EXPECT_NEAR(y1(0, 0), std::exp(0.1) - std::exp(0), 1e-3);
}

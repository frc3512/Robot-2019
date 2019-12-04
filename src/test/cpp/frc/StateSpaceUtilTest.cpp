/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "frc/StateSpaceUtil.h"
#include "frc/system/RungeKutta.h"

TEST(StateSpaceUtilTest, CostParameterPack) {
  auto mat = frc::MakeCostMatrix(1.0, 2.0, 3.0);
  EXPECT_NEAR(mat(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(mat(0, 1), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 0), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 1), 1.0 / 4.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(2, 2), 1.0 / 9.0, 1e-3);
}

TEST(StateSpaceUtilTest, CostArray) {
  auto mat = frc::MakeCostMatrix<3>({1.0, 2.0, 3.0});
  EXPECT_NEAR(mat(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(mat(0, 1), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 0), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 1), 1.0 / 4.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(2, 2), 1.0 / 9.0, 1e-3);
}

TEST(StateSpaceUtilTest, CovParameterPack) {
  auto mat = frc::MakeCovMatrix(1.0, 2.0, 3.0);
  EXPECT_NEAR(mat(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(mat(0, 1), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 0), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 1), 4.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(2, 2), 9.0, 1e-3);
}

TEST(StateSpaceUtilTest, CovArray) {
  auto mat = frc::MakeCovMatrix<3>({1.0, 2.0, 3.0});
  EXPECT_NEAR(mat(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(mat(0, 1), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 0), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 1), 4.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(0, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(1, 2), 0.0, 1e-3);
  EXPECT_NEAR(mat(2, 2), 9.0, 1e-3);
}

class DiscretizationTest : public testing::Test {
 public:
  DiscretizationTest() {
    // Create a trivial second-order system.
    contA << 0, 1, 0, 0;
    contB << 0, 1;
    contQ << 1, 0, 0, 1;
  }

 protected:
  Eigen::Matrix<double, 2, 2> contA;
  Eigen::Matrix<double, 2, 1> contB;
  Eigen::Matrix<double, 2, 2> contQ;
};

// Check that for a simple second-order system that we can easily analyze
// analytically,
TEST_F(DiscretizationTest, DiscretizeAB) {
  Eigen::Matrix<double, 2, 1> x0;
  x0 << 1, 1;
  Eigen::Matrix<double, 1, 1> u;
  u << 1;
  Eigen::Matrix<double, 2, 2> discA;
  Eigen::Matrix<double, 2, 1> discB;

  frc::DiscretizeAB(contA, contB, 1_s, &discA, &discB);
  Eigen::Matrix<double, 2, 1> x1Discrete = discA * x0 + discB * u;

  // We now have pos = vel = accel = 1, which should give us:
  Eigen::Matrix<double, 2, 1> x1Truth;
  x1Truth(1, 0) = x0(1, 0) + 1.0 * u(0, 0);
  x1Truth(0, 0) = x0(0, 0) + 1.0 * x0(1, 0) + 0.5 * u(0, 0);

  EXPECT_EQ(x1Truth, x1Discrete);
}

// Test that the discrete approximation of Q is roughly equal to
// integral from 0 to dt of e^(A tau) Q e^(A.T tau) dtau
TEST_F(DiscretizationTest, DiscretizeQ) {
  Eigen::Matrix<double, 2, 2> discQ;
  discQ = frc::DiscretizeQ(contA, contQ, 1_s);

  Eigen::Matrix<double, 2, 2> discQIntegrated = frc::RungeKuttaTimeVarying<
      std::function<Eigen::Matrix<double, 2, 2>(
          units::second_t, const Eigen::Matrix<double, 2, 2>&)>,
      Eigen::Matrix<double, 2, 2>>(
      [this](units::second_t t, const Eigen::Matrix<double, 2, 2>&) {
        return Eigen::Matrix<double, 2, 2>(
            (contA * t.to<double>()).exp() * contQ *
            (contA.transpose() * t.to<double>()).exp());
      },
      Eigen::Matrix<double, 2, 2>::Zero(), 0_s, 1_s);

  EXPECT_LT((discQIntegrated - discQ).norm(), 1e-10)
      << "Expected these to be nearly equal:\ndiscQ:\n"
      << discQ << "\ndiscQIntegrated:\n"
      << discQIntegrated;
}

// Test that the "fast" discretization produces nearly identical results.
TEST_F(DiscretizationTest, DiscretizeAQ) {
  Eigen::Matrix<double, 2, 2> discQ;
  Eigen::Matrix<double, 2, 2> discQFast;
  Eigen::Matrix<double, 2, 2> discA;
  Eigen::Matrix<double, 2, 2> discAFast;
  Eigen::Matrix<double, 2, 1> discB;

  const auto dt = 1_s;
  discQ = frc::DiscretizeQ(contA, contQ, dt);
  frc::DiscretizeAB(contA, contB, dt, &discA, &discB);
  frc::DiscretizeAQ(contA, contQ, dt, &discAFast, &discQFast);

  EXPECT_LT((discQ - discQFast).norm(), 1e-20);
  EXPECT_LT((discA - discAFast).norm(), 1e-20);
}

// Test that DiscretizeR() works
TEST_F(DiscretizationTest, DiscretizeR) {
  Eigen::Matrix<double, 2, 2> contR;
  contR << 2.0, 0.0, 0.0, 1.0;

  Eigen::Matrix<double, 2, 2> discRTruth;
  discRTruth << 4.0, 0.0, 0.0, 2.0;

  Eigen::Matrix<double, 2, 2> discR = frc::DiscretizeR(contR, 500_ms);

  EXPECT_LT((discRTruth - discR).norm(), 1e-10)
      << "Expected these to be nearly equal:\ndiscR:\n"
      << discR << "\ndiscRTruth:\n"
      << discRTruth;
}

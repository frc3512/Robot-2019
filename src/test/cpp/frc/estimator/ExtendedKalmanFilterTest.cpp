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

#include "frc/MatrixUtil.h"
#include "frc/estimator/ExtendedKalmanFilter.h"
#include "frc/system/plant/DCMotor.h"

namespace {
Eigen::Matrix<double, 5, 1> Dynamics(const Eigen::Matrix<double, 5, 1>& x,
                                     const Eigen::Matrix<double, 2, 1>& u) {
  auto motors = frc::DCMotor::CIM(2);

  // constexpr double Glow = 15.32;       // Low gear ratio
  constexpr double Ghigh = 7.08;       // High gear ratio
  constexpr auto rb = 0.8382_m / 2.0;  // Robot radius
  constexpr auto r = 0.0746125_m;      // Wheel radius
  constexpr auto m = 63.503_kg;        // Robot mass
  constexpr auto J = 5.6_kg_sq_m;      // Robot moment of inertia

  auto C1 = -std::pow(Ghigh, 2) * motors.Kt /
            (motors.Kv * motors.R * units::math::pow<2>(r));
  auto C2 = Ghigh * motors.Kt / (motors.R * r);
  auto k1 = (1 / m + units::math::pow<2>(rb) / J);
  auto k2 = (1 / m - units::math::pow<2>(rb) / J);

  units::meters_per_second_t vl{x(3, 0)};
  units::meters_per_second_t vr{x(4, 0)};
  units::volt_t Vl{u(0, 0)};
  units::volt_t Vr{u(1, 0)};

  Eigen::Matrix<double, 5, 1> result;
  auto v = 0.5 * (vl + vr);
  result(0, 0) = v.to<double>() * std::cos(x(2, 0));
  result(1, 0) = v.to<double>() * std::sin(x(2, 0));
  result(2, 0) = ((vr - vl) / (2.0 * rb)).to<double>();
  result(3, 0) =
      k1.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
      k2.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>());
  result(4, 0) =
      k2.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
      k1.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>());
  return result;
}

Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
  static_cast<void>(u);
  Eigen::Matrix<double, 3, 1> y;
  y << x(2, 0), x(3, 0), x(4, 0);
  return y;
}

Eigen::Matrix<double, 5, 1> GlobalMeasurementModel(
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
  static_cast<void>(u);
  Eigen::Matrix<double, 5, 1> y;
  y << x(0, 0), x(1, 0), x(2, 0), x(3, 0), x(4, 0);
  return y;
}
}  // namespace

TEST(ExtendedKalmanFilter, Init) {
  frc::ExtendedKalmanFilter<5, 2, 3> observer{
      Dynamics, LocalMeasurementModel,
      std::array<double, 5>{0.5, 0.5, 10.0, 1.0, 1.0},
      std::array<double, 3>{0.0001, 0.01, 0.01}};
  Eigen::Matrix<double, 2, 1> u;
  u << 12.0, 12.0;
  observer.Predict(u, 0.00505_s);

  Eigen::Matrix<double, 3, 1> localY =
      LocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  Eigen::Matrix<double, 5, 1> globalY =
      GlobalMeasurementModel(observer.Xhat(), u);
  auto R =
      frc::MakeCovMatrix(std::array<double, 5>{0.01, 0.01, 0.0001, 0.01, 0.01});
  observer.Correct<5>(u, globalY, GlobalMeasurementModel, R);
}

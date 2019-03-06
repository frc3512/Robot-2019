/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>
#include <units/units.h>

namespace frc {

/**
 * Performs Runge-Kutta integration (4th order).
 *
 * @param f  The function to integrate. It must take one argument x.
 * @param x  The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T RungeKutta(F&& f, T x, units::second_t dt) {
  const auto halfDt = 0.5 * dt;
  T k1 = f(x);
  T k2 = f(x + k1 * halfDt.to<double>());
  T k3 = f(x + k2 * halfDt.to<double>());
  T k4 = f(x + k3 * dt.to<double>());
  return x + dt.to<double>() / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs Runge Kutta integration (4th order).
 *
 * @param f  The function to integrate. It must take two arguments x and u.
 * @param x  The initial value of x.
 * @param u  The value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T RungeKutta(F&& f, T x, U u, units::second_t dt) {
  const auto halfDt = 0.5 * dt;
  T k1 = f(x, u);
  T k2 = f(x + k1 * halfDt.to<double>(), u);
  T k3 = f(x + k2 * halfDt.to<double>(), u);
  T k4 = f(x + k3 * dt.to<double>(), u);
  return x + dt.to<double>() / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace frc

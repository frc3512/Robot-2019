/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <functional>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <drake/math/discrete_algebraic_riccati_equation.h>

#include "frc/MatrixUtil.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/system/RungeKutta.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

template <int States, int Inputs, int Outputs>
class ExtendedKalmanFilter {
 public:
  /**
   * Constructs an extended Kalman filter.
   *
   * @param f                  A vector-valued function of x and u that returns
   *                           the derivative of the state vector.
   * @param h                  A vector-valued function of x and u that returns
   *                           the measurement vector.
   * @param stateStdDevs       Standard deviations of model states.
   * @param measurementStdDevs Standard deviations of measurements.
   */
  ExtendedKalmanFilter(std::function<Vector<States>(const Vector<States>&,
                                                    const Vector<Inputs>&)>
                           f,
                       std::function<Vector<Outputs>(const Vector<States>&,
                                                     const Vector<Inputs>&)>
                           h,
                       const std::array<double, States>& stateStdDevs,
                       const std::array<double, Outputs>& measurementStdDevs)
      : m_f(f), m_h(h) {
    m_Q = MakeCovMatrix(stateStdDevs);
    m_R = MakeCovMatrix(measurementStdDevs);

    Reset();

    const Eigen::Matrix<double, States, States> A =
        NumericalJacobianX<States, States, Inputs>(
            m_f, m_xHat, Eigen::Matrix<double, Inputs, 1>::Zero());
    const Eigen::Matrix<double, Outputs, States> C =
        NumericalJacobianX<Outputs, States, Inputs>(
            m_h, m_xHat, Eigen::Matrix<double, Inputs, 1>::Zero());

    // FIXME: Use CARE solver instead since A is from continuous system?
    m_initP = drake::math::DiscreteAlgebraicRiccatiEquation(
        A.transpose(), C.transpose(), m_Q, m_R);
    m_P = m_initP;
  }

  /**
   * Returns the state estimate x-hat.
   */
  const Eigen::Matrix<double, States, 1>& Xhat() const { return m_xHat; }

  /**
   * Returns an element of the state estimate x-hat.
   *
   * @param i Row of x-hat.
   */
  double Xhat(int i) const { return m_xHat(i, 0); }

  /**
   * Set initial state estimate x-hat.
   *
   * @param xHat The state estimate x-hat.
   */
  void SetXhat(const Eigen::Matrix<double, States, 1>& xHat) { m_xHat = xHat; }

  /**
   * Set an element of the initial state estimate x-hat.
   *
   * @param i     Row of x-hat.
   * @param value Value for element of x-hat.
   */
  void SetXhat(int i, double value) { m_xHat(i, 0) = value; }

  /**
   * Resets the observer.
   */
  void Reset() {
    m_xHat.setZero();
    m_P = m_initP;
  }

  /**
   * Project the model into the future with a new control input u.
   *
   * @param u  New control input from controller.
   * @param dt Timestep for prediction.
   */
  void Predict(const Eigen::Matrix<double, Inputs, 1>& u, units::second_t dt) {
    const Eigen::Matrix<double, States, States> A =
        NumericalJacobianX<States, States, Inputs>(m_f, m_xHat, u);

    m_xHat = RungeKutta(m_f, m_xHat, u, dt);
    m_P = A * m_P * A.transpose() + m_Q;
  }

  /**
   * Correct the state estimate x-hat using the measurements in y.
   *
   * @param u Same control input used in the predict step.
   * @param y Measurement vector.
   */
  void Correct(const Eigen::Matrix<double, Inputs, 1>& u,
               const Eigen::Matrix<double, Outputs, 1>& y) {
    Correct(u, y, m_h, m_R);
  }

  /**
   * Correct the state estimate x-hat using the measurements in y.
   *
   * This is useful for when the measurements available during a timestep's
   * Correct() call vary. The h(x, u) passed to the constructor is used if one
   * is not provided (the two-argument version of this function).
   *
   * @param u Same control input used in the predict step.
   * @param y Measurement vector.
   * @param h A vector-valued function of x and u that returns
   *          the measurement vector.
   * @param R Measurement noise covariance matrix.
   */
  template <int Rows>
  void Correct(
      const Eigen::Matrix<double, Inputs, 1>& u,
      const Eigen::Matrix<double, Rows, 1>& y,
      std::function<Vector<Rows>(const Vector<States>&, const Vector<Inputs>&)>
          h,
      const Eigen::Matrix<double, Rows, Rows>& R) {
    const Eigen::Matrix<double, Rows, States> C =
        NumericalJacobianX<Rows, States, Inputs>(h, m_xHat, u);

    auto S = C * m_P * C.transpose() + R;

    // We want to put K = PC^T S^-1 into Ax = b form so we can solve it more
    // efficiently.
    //
    // K = PC^T S^-1
    // KS = PC^T
    // (KS)^T = (PC^T)^T
    // S^T K^T = CP^T
    //
    // The solution of Ax = b can be found via x = A.solve(b).
    //
    // K^T = S^T.solve(CP^T)
    // K = (S^T.solve(CP^T))^T
    auto K = S.transpose().ldlt().solve(C * m_P.transpose()).transpose();

    m_xHat += K * (y - h(m_xHat, u));
    m_P = (Eigen::Matrix<double, States, States>::Identity() - K * C) * m_P;
  }

 private:
  std::function<Vector<States>(const Vector<States>&, const Vector<Inputs>&)>
      m_f;
  std::function<Vector<Outputs>(const Vector<States>&, const Vector<Inputs>&)>
      m_h;
  Eigen::Matrix<double, States, 1> m_xHat;
  Eigen::Matrix<double, States, States> m_P;
  Eigen::Matrix<double, States, States> m_Q;
  Eigen::Matrix<double, Outputs, Outputs> m_R;

  Eigen::Matrix<double, States, States> m_initP;
};

}  // namespace frc

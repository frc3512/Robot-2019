/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <cmath>
#include <type_traits>

#include <Eigen/Core>
#include <units/units.h>

#if __cplusplus < 201703L
namespace std {
template <class...>
struct conjunction : std::true_type {};
template <class B1>
struct conjunction<B1> : B1 {};
template <class B1, class... Bn>
struct conjunction<B1, Bn...>
    : std::conditional_t<bool(B1::value), conjunction<Bn...>, B1> {};

template <class... B>
constexpr bool conjunction_v = conjunction<B...>::value;
}  // namespace std
#endif

namespace frc {
namespace detail {

#if __cplusplus >= 201703L
template <typename Matrix, typename T, typename... Ts>
void CostMatrixImpl(Matrix& result, T elem, Ts... elems) {
  result(result.rows() - (sizeof...(Ts) + 1)) = 1.0 / std::pow(elem, 2);
  if constexpr (sizeof...(Ts) > 0) {
    CostMatrixImpl(result, elems...);
  }
}
#else
template <typename Matrix>
void CostMatrixImpl(Matrix& result) {}

template <typename Matrix, typename T, typename... Ts>
void CostMatrixImpl(Matrix& result, T elem, Ts... elems) {
  result(result.rows() - (sizeof...(Ts) + 1)) = 1.0 / std::pow(elem, 2);
  CostMatrixImpl(result, elems...);
}
#endif

#if __cplusplus >= 201703L
template <typename Matrix, typename T, typename... Ts>
void CovMatrixImpl(Matrix& result, T elem, Ts... elems) {
  result(result.rows() - (sizeof...(Ts) + 1)) = std::pow(elem, 2);
  if constexpr (sizeof...(Ts) > 0) {
    CovMatrixImpl(result, elems...);
  }
}
#else
template <typename Matrix>
void CovMatrixImpl(Matrix& result) {}

template <typename Matrix, typename T, typename... Ts>
void CovMatrixImpl(Matrix& result, T elem, Ts... elems) {
  result(result.rows() - (sizeof...(Ts) + 1)) = std::pow(elem, 2);
  CovMatrixImpl(result, elems...);
}
#endif
}  // namespace detail

/**
 * Creates a cost matrix from the given vector for use with LQR.
 *
 * The cost matrix is constructed using Bryson's rule. The inverse square of
 * each element in the input is taken and placed on the cost matrix diagonal.
 *
 * @param costs An array. For a Q matrix, its elements are the maximum allowed
 *              excursions of the states from the reference. For an R matrix,
 *              its elements are the maximum allowed excursions of the control
 *              inputs from no actuation.
 * @return State excursion or control effort cost matrix.
 */
template <typename... Ts, typename = std::enable_if_t<
                              std::conjunction_v<std::is_same<double, Ts>...>>>
Eigen::Matrix<double, sizeof...(Ts), sizeof...(Ts)> MakeCostMatrix(
    Ts... costs) {
  Eigen::DiagonalMatrix<double, sizeof...(Ts)> result;
  detail::CostMatrixImpl(result.diagonal(), costs...);
  return result;
}

/**
 * Creates a covariance matrix from the given vector for use with Kalman
 * filters.
 *
 * Each element is squared and placed on the covariance matrix diagonal.
 *
 * @param stdDevs An array. For a Q matrix, its elements are the standard
 *                deviations of each state from how the model behaves. For an R
 *                matrix, its elements are the standard deviations for each
 *                output measurement.
 * @return Process noise or measurement noise covariance matrix.
 */
template <typename... Ts, typename = std::enable_if_t<
                              std::conjunction_v<std::is_same<double, Ts>...>>>
Eigen::Matrix<double, sizeof...(Ts), sizeof...(Ts)> MakeCovMatrix(
    Ts... stdDevs) {
  Eigen::DiagonalMatrix<double, sizeof...(Ts)> result;
  detail::CovMatrixImpl(result.diagonal(), stdDevs...);
  return result;
}

/**
 * Creates a cost matrix from the given vector for use with LQR.
 *
 * The cost matrix is constructed using Bryson's rule. The inverse square of
 * each element in the input is taken and placed on the cost matrix diagonal.
 *
 * @param costs An array. For a Q matrix, its elements are the maximum allowed
 *              excursions of the states from the reference. For an R matrix,
 *              its elements are the maximum allowed excursions of the control
 *              inputs from no actuation.
 * @return State excursion or control effort cost matrix.
 */
template <size_t N>
Eigen::Matrix<double, N, N> MakeCostMatrix(const std::array<double, N>& costs) {
  Eigen::DiagonalMatrix<double, N> result;
  auto& diag = result.diagonal();
  for (size_t i = 0; i < N; ++i) {
    diag(i) = 1.0 / std::pow(costs[i], 2);
  }
  return result;
}

/**
 * Creates a covariance matrix from the given vector for use with Kalman
 * filters.
 *
 * Each element is squared and placed on the covariance matrix diagonal.
 *
 * @param stdDevs An array. For a Q matrix, its elements are the standard
 *                deviations of each state from how the model behaves. For an R
 *                matrix, its elements are the standard deviations for each
 *                output measurement.
 * @return Process noise or measurement noise covariance matrix.
 */
template <size_t N>
Eigen::Matrix<double, N, N> MakeCovMatrix(
    const std::array<double, N>& stdDevs) {
  Eigen::DiagonalMatrix<double, N> result;
  auto& diag = result.diagonal();
  for (size_t i = 0; i < N; ++i) {
    diag(i) = std::pow(stdDevs[i], 2);
  }
  return result;
}

/**
 * Returns a discretized version of the provided continuous process noise
 * covariance matrix.
 *
 * @param Q  Continuous process noise covariance matrix.
 * @param dt Discretization timestep.
 */
template <int States>
Eigen::Matrix<double, States, States> DiscretizeProcessNoiseCov(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, States>& Q, units::second_t dt) {
  Eigen::Matrix<double, 2 * States, 2 * States> Mgain;
  Mgain.setZero();

  // Set up the matrix M = [[-A, Q], [0, A.T]]
  Mgain.template block<States, States>(0, 0) = -A;
  Mgain.template block<States, States>(0, States) = Q;
  Mgain.template block<States, States>(States, States) = A.transpose();

  Eigen::Matrix<double, 2 * States, 2 * States> phi =
      (Mgain * dt.to<double>()).exp();

  // Phi12 = phi[0:States,        States:2*States]
  // Phi22 = phi[States:2*States, States:2*States]
  Eigen::Matrix<double, States, States> phi12 =
      phi.block(0, States, States, States);
  Eigen::Matrix<double, States, States> phi22 =
      phi.block(States, States, States, States);

  return phi22.transpose() * phi12;
}

/**
 * Returns a discretized version of the provided continuous measurement noise
 * covariance matrix.
 *
 * @param R  Continuous measurement noise covariance matrix.
 * @param dt Discretization timestep.
 */
template <int Outputs>
Eigen::Matrix<double, Outputs, Outputs> DiscretizeMeasurementNoiseCov(
    const Eigen::Matrix<double, Outputs, Outputs>& R, units::second_t dt) {
  return R / dt.to<double>();
}

}  // namespace frc

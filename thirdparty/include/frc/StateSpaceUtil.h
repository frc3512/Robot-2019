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
#include <unsupported/Eigen/MatrixFunctions>

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
 * Discretizes the given continuous A matrix.
 *
 * @param contA Continuous system matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 */
template <int States>
void DiscretizeA(const Eigen::Matrix<double, States, States>& contA,
                 units::second_t dt,
                 Eigen::Matrix<double, States, States>* discA) {
  *discA = (contA * dt.to<double>()).exp();
}

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template <int States, int Inputs>
void DiscretizeAB(const Eigen::Matrix<double, States, States>& contA,
                  const Eigen::Matrix<double, States, Inputs>& contB,
                  units::second_t dt,
                  Eigen::Matrix<double, States, States>* discA,
                  Eigen::Matrix<double, States, Inputs>* discB) {
  // Matrices are blocked here to minimize matrix exponentiation calculations
  Eigen::Matrix<double, States + Inputs, States + Inputs> Mcont;
  Mcont.setZero();
  Mcont.template block<States, States>(0, 0) = contA * dt.to<double>();
  Mcont.template block<States, Inputs>(0, States) = contB * dt.to<double>();

  // Discretize A and B with the given timestep
  Eigen::Matrix<double, States + Inputs, States + Inputs> Mdisc = Mcont.exp();
  *discA = Mdisc.template block<States, States>(0, 0);
  *discB = Mdisc.template block<States, Inputs>(0, States);
}

/**
 * Discretizes the given continuous A and Q matrices.
 *
 * @param contA Continuous system matrix.
 * @param contQ Continuous process noise covariance matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discQ Storage for discrete process noise covariance matrix.
 */
template <int States>
void DiscretizeAQ(const Eigen::Matrix<double, States, States>& contA,
                  const Eigen::Matrix<double, States, States>& contQ,
                  units::second_t dt,
                  Eigen::Matrix<double, States, States>* discA,
                  Eigen::Matrix<double, States, States>* discQ) {
  // Make Q symmetric if it isn't already
  Eigen::Matrix<double, States, States> Qtemp =
      (contQ + contQ.transpose()) / 2.0;

  Eigen::Matrix<double, 2 * States, 2 * States> M;
  M.setZero();

  // Set up the matrix M = [[-A, Q], [0, A.T]]
  M.template block<States, States>(0, 0) = -contA;
  M.template block<States, States>(0, States) = Qtemp;
  M.template block<States, States>(States, States) = contA.transpose();

  Eigen::Matrix<double, 2 * States, 2 * States> phi =
      (M * dt.to<double>()).exp();

  // Phi12 = phi[0:States,        States:2*States]
  // Phi22 = phi[States:2*States, States:2*States]
  Eigen::Matrix<double, States, States> phi12 =
      phi.block(0, States, States, States);
  Eigen::Matrix<double, States, States> phi22 =
      phi.block(States, States, States, States);

  *discA = phi22.transpose();

  Qtemp = *discA * phi12;

  // Make Q symmetric if it isn't already
  *discQ = (Qtemp + Qtemp.transpose()) / 2.0;
}

/**
 * Discretizes the given continuous A and Q matrices.
 *
 * Rather than solving a 2N x 2N matrix exponential like in DiscretizeAQ()
 * (which is expensive), we take advantage of the structure of the block matrix
 * of A and Q.
 *
 * The exponential of A*t, which is only N x N, is relatively cheap.
 * 2) The upper-right quarter of the 2N x 2N matrix, which we can approximate
 *    using a taylor series to several terms and still be substantially cheaper
 *    than taking the big exponential.
 *
 * @param contA Continuous system matrix.
 * @param contQ Continuous process noise covariance matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discQ Storage for discrete process noise covariance matrix.
 */
template <int States>
void DiscretizeAQTaylor(const Eigen::Matrix<double, States, States>& contA,
                        const Eigen::Matrix<double, States, States>& contQ,
                        units::second_t dt,
                        Eigen::Matrix<double, States, States>* discA,
                        Eigen::Matrix<double, States, States>* discQ) {
  // Make Q symmetric if it isn't already
  Eigen::Matrix<double, States, States> Qtemp =
      (contQ + contQ.transpose()) / 2.0;

  Eigen::Matrix<double, States, States> lastTerm = Qtemp;
  double lastCoeff = dt.to<double>();
  const Eigen::Matrix<double, States, States> At = contA.transpose();
  Eigen::Matrix<double, States, States> Atn = At;
  Eigen::Matrix<double, States, States> phi12 = lastTerm * lastCoeff;
  Eigen::Matrix<double, States, States> phi22 = At.Identity() + Atn * lastCoeff;

  // i = 6 i.e. 6th order should be enough precision
  for (int i = 2; i < 6; ++i) {
    Eigen::Matrix<double, States, States> nextTerm =
        -contA * lastTerm + Qtemp * Atn;
    lastCoeff *= dt.to<double>() / static_cast<double>(i);
    phi12 += nextTerm * lastCoeff;

    lastTerm = nextTerm;

    Atn *= At;
    phi22 += lastCoeff * Atn;
  }
  *discA = phi22.transpose();

  Qtemp = *discA * phi12;

  // Make Q symmetric if it isn't already
  *discQ = (Qtemp + Qtemp.transpose()) / 2.0;
}

/**
 * Returns a discretized version of the provided continuous measurement noise
 * covariance matrix.
 *
 * @param R  Continuous measurement noise covariance matrix.
 * @param dt Discretization timestep.
 */
template <int Outputs>
Eigen::Matrix<double, Outputs, Outputs> DiscretizeR(
    const Eigen::Matrix<double, Outputs, Outputs>& R, units::second_t dt) {
  return R / dt.to<double>();
}

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A,B) is stabilizable if and only if the uncontrollable eigenvalues of
 * A, if any, have absolute values less than one, where an eigenvalue is
 * uncontrollable if rank(lambda * I - A, B) < n.
 *
 * @param A System matrix.
 * @param B Input matrix.
 */
bool IsStabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                    const Eigen::Ref<const Eigen::MatrixXd>& B);

}  // namespace frc

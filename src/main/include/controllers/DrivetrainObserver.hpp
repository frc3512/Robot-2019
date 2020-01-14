// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/units.h>

namespace frc3512 {

/**
 * This is a shim around frc::DifferentialDriveOdometry providing a similar
 * interface as frc::ExtendedKalmanFilter. This makes the EKF a drop-in
 * replacement for when that's working.
 */
class DrivetrainObserver {
public:
    static constexpr int States = 5;
    static constexpr int Inputs = 2;
    static constexpr int Outputs = 5;

    /**
     * Returns the error covariance matrix P.
     */
    Eigen::Matrix<double, States, States> P() const {
        return Eigen::Matrix<double, States, States>::Zero();
    }

    /**
     * Returns an element of the error covariance matrix P.
     *
     * @param i Row of P.
     * @param j Column of P.
     */
    double P(int i, int j) const { return 0.0; }

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
    void SetXhat(const Eigen::Matrix<double, States, 1>& xHat) {
        m_xHat = xHat;
    }

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
    void Reset() { m_xHat.setZero(); }

    /**
     * Project the model into the future with a new control input u.
     *
     * @param u  New control input from controller.
     * @param dt Timestep for prediction.
     */
    void Predict(const Eigen::Matrix<double, Inputs, 1>& u,
                 units::second_t dt) {}

    /**
     * Correct the state estimate x-hat using the measurements in y.
     *
     * @param u Same control input used in the predict step.
     * @param y Measurement vector of [theta, v_l, v_r, x_l, x_r].
     */
    void Correct(const Eigen::Matrix<double, Inputs, 1>& u,
                 const Eigen::Matrix<double, Outputs, 1>& y) {
        m_odometry.Update(units::radian_t{y(0, 0)}, units::meter_t{y(3, 0)},
                          units::meter_t{y(4, 0)});
        m_xHat(0, 0) = m_odometry.GetPose().Translation().X().to<double>();
        m_xHat(1, 0) = m_odometry.GetPose().Translation().Y().to<double>();
        m_xHat(2, 0) = m_odometry.GetPose().Rotation().Radians().to<double>();
        m_xHat(3, 0) = y(1, 0);
        m_xHat(4, 0) = y(2, 0);
    }

private:
    frc::DifferentialDriveOdometry m_odometry{0_rad};
    Eigen::Matrix<double, States, 1> m_xHat;
};

}  // namespace frc3512

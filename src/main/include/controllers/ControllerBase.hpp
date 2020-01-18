// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>

namespace frc3512 {

template <int States, int Inputs, int Outputs>
class ControllerBase {
public:
    /**
     * Returns the reference vector.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetReferences() const = 0;

    /**
     * Returns the state vector.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetStates() const = 0;

    /**
     * Returns the input vector.
     */
    virtual const Eigen::Matrix<double, Inputs, 1>& GetInputs() const = 0;

    /**
     * Returns the output vector.
     */
    virtual const Eigen::Matrix<double, Outputs, 1>& GetOutputs() const = 0;
};

}  // namespace frc3512

// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/Pose.hpp"

Pose::Pose(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Pose& Pose::operator-=(const Pose& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    theta -= rhs.theta;
    return *this;
}

Pose Pose::operator-(const Pose& rhs) {
    Pose& ret = *this;
    ret -= rhs;
    return ret;
}

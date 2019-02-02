// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/TrajectoryPoint.hpp"

TrajectoryPoint::TrajectoryPoint(double x, double y, double theta, double v,
                                 double w) {
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->v = v;
    this->w = w;
}

TrajectoryPoint& TrajectoryPoint::operator-=(const TrajectoryPoint& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    theta -= rhs.theta;
    v -= rhs.v;
    w -= rhs.w;
    return *this;
}

TrajectoryPoint TrajectoryPoint::operator-(const TrajectoryPoint& rhs) {
    TrajectoryPoint& ret = *this;
    ret -= rhs;
    return ret;
}

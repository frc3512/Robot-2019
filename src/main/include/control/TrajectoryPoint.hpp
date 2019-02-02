// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

class TrajectoryPoint {
public:
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double w = 0.0;

    TrajectoryPoint() = default;
    TrajectoryPoint(double x, double y, double theta, double v, double w);

    TrajectoryPoint(const TrajectoryPoint&) = default;
    TrajectoryPoint& operator=(const TrajectoryPoint&) = default;
    TrajectoryPoint(TrajectoryPoint&&) noexcept = default;
    TrajectoryPoint& operator=(TrajectoryPoint&&) noexcept = default;
    TrajectoryPoint& operator-=(const TrajectoryPoint& rhs);
    TrajectoryPoint operator-(const TrajectoryPoint& rhs);
};

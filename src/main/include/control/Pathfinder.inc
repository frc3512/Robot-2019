// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

template <size_t N>
Pathfinder::Pathfinder(
    const std::array<grpl::pf::path::hermite_quintic::waypoint, N>& waypoints) {
    using namespace grpl::pf;

    using hermite_t = path::hermite_quintic;

    std::vector<hermite_t> hermites;
    path::hermite_factory::generate<hermite_t>(
        waypoints.begin(), waypoints.end(), std::back_inserter(hermites),
        hermites.max_size());

    path::arc_parameterizer param;
    param.configure(0.01, 0.01);
    param.parameterize(hermites.begin(), hermites.end(),
                       std::back_inserter(m_curves), m_curves.max_size());

    // profile.set_timeslice(0.00505);
}

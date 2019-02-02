// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <grpl/pf.h>

#include <array>
#include <fstream>
#include <vector>

#include "Constants.hpp"
#include "TrajectoryPoint.hpp"

class Pathfinder {
public:
    Pathfinder() = default;

    using hermite_t = grpl::pf::path::hermite_quintic;
    template <size_t N>
    explicit Pathfinder(const std::array<hermite_t::waypoint, N>& waypoints);

    Pathfinder(Pathfinder&&) = default;
    Pathfinder& operator=(Pathfinder&&) = default;

    TrajectoryPoint GeneratePath();

private:
    double m_v;
    double m_w;

    double m_t = 0.0;

    std::vector<grpl::pf::path::augmented_arc2d> m_curves;

    grpl::pf::profile::trapezoidal m_profile;

    grpl::pf::coupled::state m_state;

    grpl::pf::coupled::configuration_state m_center{0, 0, 0};

    double G = 12.75;
    grpl::pf::transmission::dc_motor m_dualCIM{
        12.0, 5330 * 2.0 * kPi / 60.0 / G, 2 * 2.7, 2 * 131.0, 2 * 2.41 * G};
    grpl::pf::coupled::chassis m_chassis{m_dualCIM, m_dualCIM, 0.0762, 0.5,
                                         25.0};
    grpl::pf::coupled::causal_trajectory_generator m_gen;
};

#include "control/Pathfinder.inc"

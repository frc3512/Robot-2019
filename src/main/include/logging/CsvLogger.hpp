// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <chrono>
#include <fstream>
#include <string_view>

#include <units/units.h>

namespace frc3512 {

/*
 * Class to log values separated by commas
 */
class CsvLogger {
public:
    explicit CsvLogger(std::string_view filename, std::string_view valueNames);

    template <class... doubles>
    void Log(double value, doubles... values);

    template <class... doubles>
    void Log(units::second_t t, double value, doubles... values);

private:
    template <class... doubles>
    void WriteValues(double value, doubles... values);

    double Timestamp();

    std::ofstream m_logfile;
    std::chrono::steady_clock::time_point m_startTime;
};

}  // namespace frc3512

#include "logging/CsvLogger.inc"

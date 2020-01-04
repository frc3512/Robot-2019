// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <chrono>
#include <fstream>
#include <string>

#include <units/units.h>

namespace frc3512 {

/*
 * Class to log values separated by commas
 */
class CsvLogger {
public:
    explicit CsvLogger(const std::string& filename, std::string valueNames);

    template <class... doubles>
    void Log(double value, doubles... values);

    template <class... doubles>
    void Log(units::second_t t, double value, doubles... values);

private:
    template <class... doubles>
    void LogImpl(double value, doubles... values);

    void LogImpl(double value);

    double Timestamp();

    std::ofstream m_logfile;
    std::chrono::steady_clock::time_point m_startTime;
};

}  // namespace frc3512

#include "logging/CsvLogger.inc"

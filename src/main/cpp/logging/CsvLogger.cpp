// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "logging/CsvLogger.hpp"

#include <wpi/FileSystem.h>
#include <wpi/raw_ostream.h>

using namespace frc3512;

CsvLogger::CsvLogger(const std::string& filename, std::string valueNames) {
    m_startTime = std::chrono::steady_clock::now();
    int version = 0;
    while (wpi::sys::fs::exists(filename + std::to_string(version))) {
        version++;
    }
    m_logfile.open(filename + std::to_string(version));
    if (!m_logfile.is_open()) {
        wpi::errs() << filename << " has failed to open.\n";
    }
    m_logfile << valueNames << '\n';
    m_logfile.flush();
}

void CsvLogger::LogImpl(double value) {
    m_logfile << value << '\n';
    m_logfile.flush();
}

double CsvLogger::Timestamp() {
    auto duration = std::chrono::steady_clock::now() - m_startTime;
    auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return millis / 1000.0;
}

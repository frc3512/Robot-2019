// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "logging/CsvLogger.hpp"

#include <iostream>

CsvLogger::CsvLogger(const std::string& filename) : m_logfile(filename) {
    m_startTime = hal::fpga_clock::now();
}

void CsvLogger::LogImpl(double value) {
    std::cout << std::endl;
    m_logfile << value << '\n';
}

double CsvLogger::Timestamp() {
    auto duration = hal::fpga_clock::now() - m_startTime;
    auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return millis / 1000.0;
}

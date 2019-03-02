// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogFileSink.hpp"

using namespace frc3512;

LogFileSink::LogFileSink(std::string filename) {
    m_logfile.open(filename.c_str());
}

void LogFileSink::Log(LogEvent event) {
    m_logfile << event.ToFormattedString();
    m_logfile.flush();
}

// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogFileSink.hpp"

#include <frc/Filesystem.h>
#include <wpi/SmallVector.h>
#include <wpi/Twine.h>

using namespace frc3512;

LogFileSink::LogFileSink(std::string filename) {
    wpi::SmallVector<char, 64> path;
    frc::filesystem::GetOperatingDirectory(path);

    m_logfile.open((path + "/" + filename).str());
}

void LogFileSink::Log(LogEvent event) {
    m_logfile << event.ToFormattedString();
    m_logfile.flush();
}

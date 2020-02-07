// Copyright (c) 2014-2020 FRC Team 3512. All Rights Reserved.

#include "logging/LogFileSink.hpp"

#include <frc/Filesystem.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/Twine.h>

using namespace frc3512;

LogFileSink::LogFileSink(std::string filename) {
    wpi::SmallString<64> path;
    frc::filesystem::GetOperatingDirectory(path);
    wpi::sys::path::append(path, filename);

    m_logfile.open(wpi::Twine{path}.str());
}

void LogFileSink::Log(LogEvent event) {
    m_logfile << event.ToFormattedString();
    m_logfile.flush();
}

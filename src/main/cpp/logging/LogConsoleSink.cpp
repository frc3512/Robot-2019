// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogConsoleSink.hpp"

#include <wpi/raw_ostream.h>

using namespace frc3512;

void LogConsoleSink::Log(LogEvent event) {
    wpi::outs() << event.ToFormattedString();
}

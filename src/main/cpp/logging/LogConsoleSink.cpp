// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogConsoleSink.hpp"

#include <iostream>

void LogConsoleSink::Log(LogEvent event) {
    std::cout << event.ToFormattedString();
}

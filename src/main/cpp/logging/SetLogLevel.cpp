// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/SetLogLevel.hpp"

SetLogLevel::SetLogLevel(LogEvent::VerbosityLevel level) { m_level = level; }

LogStream& operator<<(LogStream& os, const SetLogLevel& in) {
    os.SetLevel(in.m_level);
    return os;
}

// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogStream.hpp"

#include "logging/LogStreambuf.hpp"

using namespace frc3512;

LogStream::LogStream(Logger& logger)
    : std::ostream(new LogStreambuf(logger)), m_logger(logger) {}

LogStream::~LogStream() { delete rdbuf(); }

void LogStream::SetLevel(LogEvent::VerbosityLevel level) {
    static_cast<LogStreambuf*>(rdbuf())->SetLevel(level);
}

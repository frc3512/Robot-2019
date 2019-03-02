// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogStreambuf.hpp"

using namespace frc3512;

LogStreambuf::LogStreambuf(Logger& logger) : m_logger(logger) {}

std::streamsize LogStreambuf::xsputn(const char* s, std::streamsize n) {
    m_buf += std::string(s, n);

    return n;
}

void LogStreambuf::SetLevel(LogEvent::VerbosityLevel level) { m_level = level; }

void LogStreambuf::Sync() {
    if (m_level != LogEvent::VERBOSE_NONE) {
        m_logger.Log(LogEvent(m_buf, m_level));
    }

    m_buf = "";
    m_level = LogEvent::VERBOSE_NONE;
}

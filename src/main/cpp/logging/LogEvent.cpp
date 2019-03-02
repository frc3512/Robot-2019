// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogEvent.hpp"

#include <iomanip>
#include <sstream>

using namespace frc3512;

LogEvent::LogEvent(std::string data, VerbosityLevel level) {
    m_level = level;
    m_timestamp = std::time(nullptr);
    m_buffer = data;
    m_initialTime = 0;
}

LogEvent::LogEvent(std::string data, VerbosityLevel level,
                   std::time_t timestamp) {
    m_level = level;
    m_timestamp = timestamp;
    m_buffer = data;
    m_initialTime = 0;
}

LogEvent::VerbosityLevel LogEvent::GetVerbosityLevel() { return m_level; }

std::time_t LogEvent::GetAbsoluteTimestamp() { return m_timestamp; }

std::time_t LogEvent::GetRelativeTimestamp() {
    if (m_initialTime == 0) return 0;

    return m_timestamp - m_initialTime;
}

std::string LogEvent::GetData() { return m_buffer; }

std::string LogEvent::ToFormattedString() {
    std::stringstream ss;

    ss << std::left << "[" << std::setw(8) << GetRelativeTimestamp() << " "
       << VerbosityLevelChar(GetVerbosityLevel()) << "] " << GetData() << "\n";

    return ss.str();
}

std::string LogEvent::VerbosityLevelString(VerbosityLevel levels) {
    std::string level_str;

    if (levels & VERBOSE_ERROR) level_str += "ERROR ";
    if (levels & VERBOSE_WARN) level_str += "WARN ";
    if (levels & VERBOSE_INFO) level_str += "INFO ";
    if (levels & VERBOSE_DEBUG) level_str += "DEBUG ";
    if (levels & VERBOSE_USER) level_str += "USER ";

    return level_str;
}

char LogEvent::VerbosityLevelChar(VerbosityLevel level) {
    switch (level) {
        case VERBOSE_ERROR:
            return 'E';
        case VERBOSE_WARN:
            return 'W';
        case VERBOSE_INFO:
            return 'I';
        case VERBOSE_DEBUG:
            return 'D';
        case VERBOSE_USER:
            return 'U';
        case VERBOSE_NONE:
        case VERBOSE_ALL:
        default:
            return 'X';
    }
}

void LogEvent::SetInitialTime(std::time_t initial) { m_initialTime = initial; }

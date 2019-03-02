// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogSinkBase.hpp"

using namespace frc3512;

bool LogSinkBase::operator==(const LogSinkBase& rhs) { return this == &rhs; }

void LogSinkBase::SetVerbosityLevels(LogEvent::VerbosityLevel levels) {
    m_verbosity = levels;
}

LogEvent::VerbosityLevel LogSinkBase::GetVerbosityLevels() const {
    return m_verbosity;
}

void LogSinkBase::EnableVerbosityLevels(LogEvent::VerbosityLevel levels) {
    m_verbosity |= levels;
}

void LogSinkBase::DisableVerbosityLevels(LogEvent::VerbosityLevel levels) {
    m_verbosity &= ~(levels);
}

bool LogSinkBase::TestVerbosityLevel(LogEvent::VerbosityLevel levels) const {
    return m_verbosity & levels;
}

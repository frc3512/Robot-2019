// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "logging/LogEvent.hpp"

namespace frc3512 {

/**
 * LogSinkBase provides a base class on which to implement various log event
 * sinks.
 *
 * A log event sink is a class which receives log events (in the form of
 * LogEvent class instances) through the LogEvent() callback function. The class
 * can then store, display, or otherwise process the event.
 */
class LogSinkBase {
public:
    virtual ~LogSinkBase() = default;

    bool operator==(const LogSinkBase& rhs);

    /**
     * The callback function called when an event is received whose verbosity
     * level matches that of the class (see TestVerbosityLevel()).
     *
     * @param event The event.
     */
    virtual void Log(LogEvent event) = 0;

    /**
     * Set the verbosity levels for which we will accept events.
     *
     * @param levels A bitfield describing the verbosity levels.
     */
    void SetVerbosityLevels(LogEvent::VerbosityLevel levels);

    /**
     * Get the verbosity levels for which we are currently accepting events.
     *
     * @return A bitfield describing the verbosity levels.
     */
    LogEvent::VerbosityLevel GetVerbosityLevels() const;

    /**
     * Begin accepting events of the specified verbosity levels.
     *
     * @param levels A bitfield describing the verbosity levels.
     */
    void EnableVerbosityLevels(LogEvent::VerbosityLevel levels);

    /**
     * Stop accepting events of the specified verbosity levels.
     *
     * @param levels A bitfield describing the verbosity levels.
     */
    void DisableVerbosityLevels(LogEvent::VerbosityLevel levels);

    /**
     * Determines whether we accept events of a specified verbosity level.
     *
     * @param levels A bitfield describing the verbosity levels to test.
     * @return A boolean describing whether we accept events of the specified
     *         verbosity level.
     */
    bool TestVerbosityLevel(LogEvent::VerbosityLevel levels) const;

private:
    LogEvent::VerbosityLevel m_verbosity = LogEvent::VERBOSE_ERROR;
};

}  // namespace frc3512

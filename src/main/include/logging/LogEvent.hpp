// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctime>
#include <string>

namespace frc3512 {

/**
 * An event to be logged with Logger::LogEvent().
 *
 * A log event consists of several parts: a text string describing the event, a
 * verbosity level determining which sinks will accept the event, and a
 * timestamp describing the time the event occurred.
 */
class LogEvent {
public:
    enum _VerbosityLevel {
        VERBOSE_NONE = 0x00,
        VERBOSE_ERROR = 0x01,
        VERBOSE_WARN = 0x02,
        VERBOSE_INFO = 0x04,
        VERBOSE_DEBUG = 0x08,
        VERBOSE_USER = 0x10,
        VERBOSE_ALL = VERBOSE_ERROR | VERBOSE_WARN | VERBOSE_INFO |
                      VERBOSE_USER | VERBOSE_DEBUG
    };

    /**
     * A type describing the verbosity level of an event. We have to use an int
     * rather than an enum so we can 'or' the flags.
     */
    using VerbosityLevel = int;

    /**
     * Create an event with a specified description string and verbosity level,
     * using the current time for the timestamp.
     *
     * @param data The string describing the event.
     * @param level The event's verbosity level.
     */
    LogEvent(std::string data, VerbosityLevel level);

    /**
     * Create an event with a specified description string, verbosity level, and
     * time.
     *
     * @param data The string describing the event.
     * @param level The event's verbosity level.
     * @param timestamp The time at which the event occurred.
     */
    LogEvent(std::string data, VerbosityLevel level, std::time_t timestamp);

    virtual ~LogEvent() = default;

    /**
     * Retrieve the event's verbosity level.
     *
     * @return The verbosity level of the event.
     */
    VerbosityLevel GetVerbosityLevel();

    /**
     * Retrieve the time the event occurred as a time in seconds since the
     * Epoch.
     *
     * @return A normal POSIX format time.
     */
    std::time_t GetAbsoluteTimestamp();

    /**
     * Retrieve the time the event occurred as a time in seconds since the
     * logging engine was initialized.
     *
     * Note that this function will always return zero for events that have not
     * touched the Logger class because Logger calls SetInitialTime(), which is
     * neccessary for GetRelativeTimestamp() to work.
     *
     * @return The time in seconds since the initialization of the Logger class
     *         instance.
     */
    std::time_t GetRelativeTimestamp();

    /**
     * Retrieves the string describing the event.
     *
     * @return The string describing the event.
     */
    std::string GetData();

    /**
     * Formats the information contained in the event in a printable string.
     *
     * The string consists of the relative timestamp of the event (see
     * GetRelativeTimestamp()), followed by a character describing the type of
     * event (see VerbosityLevelChar()) enclosed in square brackets ("[]"). The
     * event description (see GetData()) constitutes the remainder of the
     * string.
     *
     * @return A formatted string describing various attributes of the event.
     * \ref GetRelativeTimestamp()
     * \ref VerbosityLevelChar()
     * \ref GetData()
     */
    std::string ToFormattedString();

    /**
     * Returns a textual representation of the specified verbosity levels.
     *
     * For the following verbosity levels, the textual representation
     * corresponds exactly to second part of the enum symbol:
     *
     *   VERBOSE_ERROR  "ERROR"
     *   VERBOSE_WARN   "WARN"
     *   VERBOSE_INFO   "INFO"
     *   VERBOSE_USER   "USER"
     *
     * If the specified VerbosityLevel has more than one flag set, the string
     * returned will consist of one or more of the above strings separated by
     * the space character.
     *
     * @param level The verbosity level from which to generate a string.
     * @return A string describing the specified verbosity level.
     */
    static std::string VerbosityLevelString(VerbosityLevel level);

    /**
     * Returns a single character representing the specified verbosity level.
     *
     * The verbosity levels are mapped to characters as follows:
     *
     *   VERBOSE_ERROR  'E'
     *   VERBOSE_WARN   'W'
     *   VERBOSE_INFO   'I'
     *   VERBOSE_USER   'U'
     *
     * The character 'X' is returned for all other values.
     *
     * @param level The verbosity level from which to generate a string.
     * @return A string describing the specified verbosity level.
     */
    static char VerbosityLevelChar(VerbosityLevel level);

    /**
     * Called by the Logger class. Sets the initial time used by the
     * GetRelativeTimestamp() function.
     *
     * @param initial The initial time on which to base relative timestamps.
     */
    void SetInitialTime(std::time_t initial);

private:
    VerbosityLevel m_level;
    std::time_t m_timestamp;
    std::string m_buffer;
    std::time_t m_initialTime;
};

}  // namespace frc3512

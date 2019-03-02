// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <iostream>

#include "logging/LogEvent.hpp"
#include "logging/Logger.hpp"

namespace frc3512 {

/**
 * A subclass of std::ostream for logging messages with a Logger class.
 *
 * To log a message, one must either pipe a SetLogLevel instance into the stream
 * or call the SetLevel() function of the class to notify the class of the log
 * level with which to log the message. After setting the verbosity level, one
 * or more strings should be piped into the stream, representing the message
 * that will be displayed.
 *
 * When the entire message description has been piped into the stream, a
 * std::flush must be piped into the stream to notify the class to log the
 * message. The verbosity level must be set each time a new message is logged.
 * If the verbosity level is not set, the message will be dropped. The current
 * time will always be used as the timestamp for the log message.
 */
class LogStream : public std::ostream {
public:
    /**
     * The constructor.
     *
     * @param logger The logger class instance to which to log messages.
     */
    explicit LogStream(Logger& logger);
    virtual ~LogStream();

    /**
     * Sets the verbosity level with which to log the current message.
     *
     * @param level The verbosity level with which to log the current message.
     */
    void SetLevel(LogEvent::VerbosityLevel level);

private:
    Logger& m_logger;
};

}  // namespace frc3512

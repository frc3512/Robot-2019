// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ios>
#include <streambuf>
#include <string>

#include "logging/LogEvent.hpp"
#include "logging/Logger.hpp"

namespace frc3512 {

/**
 * An internal class used by LogStream.
 *
 * This class implements the interface with the Logger class.
 */
class LogStreambuf : public std::streambuf {
public:
    explicit LogStreambuf(Logger& logger);
    virtual ~LogStreambuf() = default;

    std::streamsize xsputn(const char* s, std::streamsize n) override;

    /**
     * Called when the stream is flushed. This can occur when the user pipes
     * std::flush or std::endl into the stream.
     *
     * This function generates an event from the accumulated information
     * (message, verbosity level, etc...) and calls the given logEvent function
     * of the given Logger class instance.
     */
    void Sync();

    /**
     * The SetLevel() function is called by LogStream::SetLevel().
     *
     * It simply stores the provided verbosity level for use when a flush event
     * occurs on the stream (causing the Sync() function to be called).
     * \see Sync()
     */
    void SetLevel(LogEvent::VerbosityLevel level);

private:
    std::string m_buf;
    Logger& m_logger;
    LogEvent::VerbosityLevel m_level = LogEvent::VERBOSE_NONE;
};

}  // namespace frc3512

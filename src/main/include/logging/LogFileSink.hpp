// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <fstream>
#include <string>

#include "logging/LogSinkBase.hpp"

namespace frc3512 {

/**
 * A file sink for the logged events.
 */
class LogFileSink : public LogSinkBase {
public:
    explicit LogFileSink(std::string filename);
    virtual ~LogFileSink() = default;

    /**
     * Write an event to the logfile.
     *
     * @param event The event to log.
     */
    void Log(LogEvent event) override;

private:
    std::ofstream m_logfile;
};

}  // namespace frc3512

// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "communications/PublishNode.hpp"
#include "logging/LogEvent.hpp"
#include "logging/LogSinkBase.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * A logging engine.
 */
class Logger : public LogSinkBase, public PublishNode, public SubsystemBase {
public:
    using LogSinkBaseList = std::vector<std::reference_wrapper<LogSinkBase>>;

    /**
     * The constructor.
     *
     * Calls ResetInitialTime().
     */
    Logger();
    virtual ~Logger() = default;

    /**
     * Logs an event to all registered LogSinkBase sinks whose verbosity levels
     * contain that of the event (see LogSinkBase::TestVerbosityLevel()).
     *
     * @param event The event to log.
     */
    void Log(LogEvent event) override;

    /**
     * Registers a sink for log events with the logging engine.
     *
     * All events with a verbosity level matching that of the sink (see
     * LogSinkBase::TestVerbosityLevel()) which are logged after the sink is
     * registered will be sent to the sink.
     *
     * @param sink The sink class instance to register.
     */
    void AddLogSink(LogSinkBase& sink);

    /**
     * De-registers a sink from the logging engine.
     *
     * @param sink The sink to de-register.
     */
    void RemoveLogSink(LogSinkBase& sink);

    /**
     * Returns a list of currently registered sinks.
     *
     * @return A list of currently registered sinks.
     */
    LogSinkBaseList ListLogSinks() const;

    /**
     * Sets the 'initial' time to the current time.
     *
     * This initial time is used to calculate an event's relative time.
     */
    void ResetInitialTime();

    /**
     * Sets the 'initial' time to the specified time.
     *
     * This initial time is used to calculate an event's relative time.
     */
    void SetInitialTime(std::time_t time);

    void ProcessMessage(const StatePacket& message) override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

private:
    LogSinkBaseList m_sinkList;
    std::time_t m_initialTime;
};

}  // namespace frc3512

// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/Logger.hpp"

#include <algorithm>

#include "Robot.hpp"

using namespace frc3512;

Logger::Logger() : Logger::PublishNode("Logger") { ResetInitialTime(); }

void Logger::Log(LogEvent event) {
    event.SetInitialTime(m_initialTime);

    for (auto sink : m_sinkList) {
        if (sink.get().TestVerbosityLevel(event.GetVerbosityLevel())) {
            sink.get().Log(event);
        }
    }
}

void Logger::AddLogSink(LogSinkBase& sink) { m_sinkList.emplace_back(sink); }

void Logger::RemoveLogSink(LogSinkBase& sink) {
    m_sinkList.erase(
        std::remove_if(m_sinkList.begin(), m_sinkList.end(),
                       [&](std::reference_wrapper<LogSinkBase> elem) -> bool {
                           return elem.get() == sink;
                       }),
        m_sinkList.end());
}

Logger::LogSinkBaseList Logger::ListLogSinks() const { return m_sinkList; }

void Logger::ResetInitialTime() { m_initialTime = std::time(nullptr); }

void Logger::SetInitialTime(std::time_t time) { m_initialTime = time; }

void Logger::ProcessMessage(const StatePacket& message) {
    Log(LogEvent(
        "StatePacket (" + message.topic + "): " + std::to_string(message.state),
        LogEvent::VERBOSE_DEBUG));
}

void Logger::ProcessMessage(const ButtonPacket& message) {
    if (message.pressed) {
        Log(LogEvent("ButtonPacket (" + message.topic +
                         "): " + std::to_string(message.button) + " Pressed",
                     LogEvent::VERBOSE_DEBUG));
    } else {
        Log(LogEvent("ButtonPacket (" + message.topic +
                         "): " + std::to_string(message.button) + " Released",
                     LogEvent::VERBOSE_DEBUG));
    }
}

void Logger::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    }
}

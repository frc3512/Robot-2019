// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "communications/PublishNode.hpp"

#include <wpi/SmallVector.h>

#include <cstring>
#include <iostream>

using namespace frc3512;

PublishNode::PublishNode(std::string nodeName) {
    m_nodeName = nodeName;
    m_thread = std::thread(&PublishNode::RunFramework, this);
}

PublishNode::~PublishNode() {
    m_isRunning = false;
    m_thread.join();
}

void PublishNode::Subscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it == publisher.m_subList.end()) {
        publisher.m_subList.push_back(this);
    }
}

void PublishNode::Unsubscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it != publisher.m_subList.end()) {
        publisher.m_subList.erase(it);
    }
}

void PublishNode::ProcessMessage(const StatePacket& message) {}

void PublishNode::ProcessMessage(const ButtonPacket& message) {}

void PublishNode::ProcessMessage(const POVPacket& message) {}

void PublishNode::RunFramework() {
    while (m_isRunning) {
        std::unique_lock<std::mutex> lock(m_mutex);

        // Waits for queue to contain messages, but does not need to wait for
        // queue to contain number of contents equal to the size of a complete
        // message due to the mutex ensuring atomic message insertions.
        m_ready.wait(lock,
                     [this] { return m_queue.size() > 0 || !m_isRunning; });

        while (m_queue.size() > 0) {
            // Pops first element out of queue for length of one whole message,
            // then pops that amount elements
            size_t msgLength = m_queue.pop_front();
            wpi::SmallVector<char, 32> message;
            for (int i = 0; i < msgLength; i++) {
                message.push_back(m_queue.pop_front());
            }

            DeserializeAndProcessMessage(message);
        }
    }
}

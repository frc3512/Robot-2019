// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include "communications/PublishNode.hpp"

#include <wpi/SmallVector.h>

using namespace frc3512;

PublishNode::PublishNode(std::string_view nodeName) {
    m_nodeName = nodeName;
    m_thread = std::thread(&PublishNode::RunFramework, this);
}

PublishNode::~PublishNode() {
    m_isRunning = false;
    m_ready.notify_all();
    m_thread.join();
}

void PublishNode::SubscribeTo(PublishNode& publisher, wpi::StringRef topic) {
    wpi::StringRef lhs;
    wpi::StringRef rhs;

    // Start at root
    auto* current = &publisher.m_subscriberTree;
    // TODO: C++17 structured bindings
    // auto [lhs, rhs] = topic.split('/');
    auto temp = topic.split('/');
    lhs = temp.first;
    rhs = temp.second;

    // While there are still parts of the topic to unpack
    while (rhs.size() > 0) {
        // If subscriber is already subscribed to a parent topic, do nothing
        if (current->data.subscribers.count(this) > 0) {
            return;
        }

        auto it = std::find_if(
            current->children.begin(), current->children.end(),
            [=](const auto& child) { return child.data.topic == lhs; });
        if (it == current->children.end()) {
            // TODO: C++17 emplace_back()
            // current = &current->children.emplace_back(lhs);
            current->children.emplace_back(lhs);
            current = &current->children.back();
        } else {
            current = &(*it);
        }

        // TODO: C++17 structured bindings
        // auto [lhs, rhs] = rhs.split('/');
        auto temp = rhs.split('/');
        lhs = temp.first;
        rhs = temp.second;
    }

    // If rhs is empty, the proper location for the subscriber was found
    if (rhs.size() == 0) {
        current->data.subscribers.insert(this);
    }

    // Unsub subscriber from all children to avoid receiving same message twice
    for (auto& child : current->children) {
        child.DFS([this](Node& node) {
            node.subscribers.erase(this);
            return true;
        });
    }
}

void PublishNode::UnsubscribeFrom(PublishNode& publisher,
                                  wpi::StringRef topic) {
    wpi::StringRef lhs;
    wpi::StringRef rhs;

    // Start at root
    auto* currentTree = &m_subscriberTree;
    // TODO: C++17 structured bindings
    // auto [lhs, rhs] = topic.split('/');
    auto temp = topic.split('/');
    lhs = temp.first;
    rhs = temp.second;

    // While there are still parts of the topic to unpack
    while (rhs.size() > 0) {
        auto it = std::find_if(
            currentTree->children.begin(), currentTree->children.end(),
            [=](const auto& child) { return child.data.topic == lhs; });
        // Found child matching topic segment, so continue searching
        if (it != currentTree->children.end()) {
            currentTree = &(*it);
        } else {
            break;
        }

        // TODO: C++17 structured bindings
        // auto [lhs, rhs] = rhs.split('/');
        auto temp = rhs.split('/');
        lhs = temp.first;
        rhs = temp.second;
    }

    // If rhs is empty, the subscriber was found, so unsub it from the current
    // tree node and all its children
    if (rhs.size() == 0) {
        currentTree->DFS([this](Node& node) {
            node.subscribers.erase(this);
            return true;
        });
    }
}

bool PublishNode::GetRawButton(const HIDPacket& message, int joystick,
                               int button) {
    if (joystick == 0) {
        return message.buttons1 & (1 << (button - 1));
    } else if (joystick == 1) {
        return message.buttons2 & (1 << (button - 1));
    } else if (joystick == 2) {
        return message.buttons3 & (1 << (button - 1));
    } else if (joystick == 3) {
        return message.buttons4 & (1 << (button - 1));
    } else {
        return false;
    }
}

void PublishNode::RunFramework() {
    while (m_isRunning) {
        std::unique_lock<wpi::mutex> lock(m_mutex);

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
            for (size_t i = 0; i < msgLength; i++) {
                message.push_back(m_queue.pop_front());
            }

            DeserializeAndProcessMessage(message);
        }
    }
}

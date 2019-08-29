// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <iostream>
#include <string>

namespace frc3512 {

template <class P>
void PublishNode::Publish(P p) {
    wpi::StringRef topic = p.topic;
    wpi::StringRef lhs;
    wpi::StringRef rhs;

    // Start at root
    auto* currentTree = &m_subscriberTree;
    // auto [lhs, rhs] = topic.split('/');
    auto temp = topic.split('/');
    lhs = temp.first;
    rhs = temp.second;

    while (rhs.size() > 0) {
        // Publish to subscribers at current tree node
        for (auto& sub : currentTree->data.subscribers) {
            sub->PushMessage(p);
            std::cout << "Publishing" << std::endl;
        }

        auto it = std::find_if(
            currentTree->children.begin(), currentTree->children.end(),
            [=](const auto& child) { return child.data.topic == lhs; });
        // Found child matching topic segment, so publish to it
        if (it != currentTree->children.end()) {
            currentTree = &(*it);
        } else {
            break;
        }

        // TODO: C++17 structured bindings
        // auto [lhs, rhs] = topic.split('/');
        auto temp = rhs.split('/');
        lhs = temp.first;
        rhs = temp.second;
    }
}

template <class P>
void PublishNode::PushMessage(P p) {
    Packet packet = p.Serialize();
    size_t len = packet.getDataSize();
    auto ptr = static_cast<const char*>(packet.getData());

    std::lock_guard lock(m_mutex);
    m_queue.push_back(len);
    for (size_t j = 0; j < len; j++) {
        m_queue.push_back(ptr[j]);
    }
    m_ready.notify_one();
}

}  // namespace frc3512

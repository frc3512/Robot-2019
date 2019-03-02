// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <frc/circular_buffer.h>

#include "Constants.hpp"
#include "communications/ButtonPacket.hpp"
#include "communications/POVPacket.hpp"
#include "communications/PacketType.hpp"
#include "communications/StatePacket.hpp"

namespace frc3512 {

/**
 * A communication layer that can pass messages between others who have
 * inherited it through a publish-subscribe architecture
 */
class PublishNode {
public:
    /**
     * Construct a PublishNode.
     *
     * @param nodeName Name of node.
     */
    explicit PublishNode(std::string nodeName = "Misc");
    virtual ~PublishNode();

    /**
     * Adds this object to the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode that this instance wants to recieve
     *                  event from.
     */
    void Subscribe(PublishNode& publisher);

    /**
     * Removes this object from the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode that this instance wants to stop
     *                  recieving event from.
     */
    void Unsubscribe(PublishNode& publisher);

    /**
     * Sends a packet to every subscriber.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <class P>
    void Publish(P p);

    /**
     * Sends a packet to the object it's called on.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <class P>
    void PushMessage(P p);

    /**
     * Processes a StatePacket.
     *
     * Users should override if this instance has a need to handle state data
     *
     * @param message The message whose member variables contain deserialized
     *                data.
     */
    virtual void ProcessMessage(const StatePacket& message);

    /**
     * Processes a ButtonPacket.
     *
     * Users should override if this instance has a need to handle joystick
     * button events.
     *
     * @param message The message whose member variables contain deserialized
     *                data.
     */
    virtual void ProcessMessage(const ButtonPacket& message);

    /**
     * Processes a POVPacket.
     *
     * Users should override if this instance has a need to handle joystick
     * POV events.
     *
     * @param message The message whose member variables contain deserialized
     *                data.
     */
    virtual void ProcessMessage(const POVPacket& message);

private:
    std::string m_nodeName;
    std::vector<PublishNode*> m_subList;
    frc::circular_buffer<char> m_queue{kNodeQueueSize};

    std::mutex m_mutex;
    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};
    std::condition_variable m_ready;

    /**
     * Blocks the thread until the queue receives at least one set of characters
     * of a message or until the node deconstructs, then processes each message.
     */
    void RunFramework();

    /**
     * Deserialize the provided message and process it via the ProcessMessage()
     * function corresponding to the message type.
     *
     * Do NOT provide an implementation for this function. messages.py generates
     * one in PacketType.cpp.
     *
     * @param message The buffer containing the message to deserialize.
     */
    void DeserializeAndProcessMessage(wpi::SmallVectorImpl<char>& message);
};

}  // namespace frc3512

#include "PublishNode.inc"

// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <wpi/SmallSet.h>
#include <wpi/StringRef.h>
#include <wpi/circular_buffer.h>
#include <wpi/condition_variable.h>

#include "SubscriptionTree.hpp"
#include "Tree.hpp"
#include "communications/PublishNodeBase.hpp"

namespace frc3512 {

/**
 * A communication layer that can pass messages between others who have
 * inherited it through a publish-subscribe architecture
 */
class PublishNode : public PublishNodeBase {
public:
    /**
     * Construct a PublishNode.
     */
    PublishNode();
    virtual ~PublishNode();

    /**
     * Adds this object to the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode from which this instance wants to
     *                  recieve events.
     * @param topic     The topic to which to subscribe.
     */
    void SubscribeTo(PublishNode& publisher, wpi::StringRef topic);

    /**
     * Removes this object from the specified PublishNode's subscriber list.
     *
     * @param publisher The PublishNode from which this instance wants to stop
     *                  recieving events.
     * @param topic     The topic from which to unsubscribe.
     */
    void UnsubscribeFrom(PublishNode& publisher, wpi::StringRef topic);

    /**
     * Get the button value (starting at button 1).
     *
     * The buttons are returned in a single 16 bit value with one bit
     * representing the state of each button. The appropriate button is returned
     * as a boolean value.
     *
     * @param message The message whose member variables contain deserialized
     *                data.
     * @param joystick  The joystick number.
     * @param button  The button number to be read (starting at 1).
     * @return The state of the button.
     */
    static bool GetRawButton(const HIDPacket& msg, int joystick, int button);

    /**
     * Sends a packet to every subscriber.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <typename P>
    void Publish(P p);

    /**
     * Sends a packet to the object it's called on.
     *
     * @param p Any packet with a Serialize() method.
     */
    template <class P>
    void PushMessage(P p);

private:
    struct Node {
        std::string topic;
        wpi::SmallSet<PublishNode*, 32> subscribers;

        explicit Node(wpi::StringRef topic) { this->topic = topic; }
    };

    static constexpr int kNodeQueueSize = 1024;

    Tree<Node> m_subscriberTree{""};
    wpi::circular_buffer<char> m_queue{kNodeQueueSize};

    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};
    wpi::condition_variable m_ready;

    /**
     * Blocks the thread until the queue receives at least one set of characters
     * of a message or until the node deconstructs, then processes each message.
     */
    void RunFramework();

    friend class PublishNodeMockTest_SubscribeTest_Test;
};

}  // namespace frc3512

#include "PublishNode.inc"

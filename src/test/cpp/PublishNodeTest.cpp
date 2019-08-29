// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>

#include "PublishNodeMock.hpp"

namespace frc3512 {

class PublishNodeMockTest : public testing::Test {
public:
    PublishNodeMock publisher;
    PublishNodeMock subscriber;
};

TEST_F(PublishNodeMockTest, SubscribeTest) {
    subscriber.SubscribeTo(publisher, "Monkeys/Chimpanzee");
    subscriber.SubscribeTo(publisher, "Monkeys/Gorilla/HomoErectus");
    /*  subscriber.m_subscriberTree.DFS([&] (auto& node){
        std::cout << "Node topic: " << node.topic << std::endl;
        for (const auto& sub : node.subscribers){
            std::cout << "  Node sub: " << sub << std::endl;
        }
        return true;
    }); */
    frc3512::ButtonPacket message{"Monkeys/Chimpanzee", 5, true};
    publisher.Publish(message);
    while (subscriber.wait) {
    }
    EXPECT_EQ(subscriber.buttonPacket.topic, "Monkeys/Chimpanzee");
}

}  // namespace frc3512

// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

#include "communications/PublishNode.hpp"

class SubscriptionTree {
public:
    explicit SubscriptionTree(std::string subtopic);

    template <class P>
    void Publish(P p);

    void Subscribe();

    void Unsubscribe();

private:
    std::string m_subtopic;
    std::vector<PublishNode*> m_subList;
    std::vector<SubscriptionTree*> m_children;
};

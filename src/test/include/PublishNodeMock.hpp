// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include "communications/PublishNode.hpp"

class PublishNodeMock : public frc3512::PublishNode {
public:
    void ProcessMessage(const frc3512::ButtonPacket& message) override {
        buttonPacket = message;
        wait = false;
    }
    frc3512::ButtonPacket buttonPacket;
    bool wait = true;

private:
};

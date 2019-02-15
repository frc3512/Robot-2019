// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Notifier.h>

/**
 * A standardized base for subsystems
 */
class SubsystemBase {
public:
    /**
     * Constructs a SubsystemBase
     *
     * @param nodeName A name to refer to the subsystem
     */
    SubsystemBase();
    virtual ~SubsystemBase() = default;

    /**
     * This function will be called asynchronously every 20 ms
     */
    virtual void SubsystemPeriodic();

private:
    frc::Notifier m_notifier{[this] { SubsystemPeriodic(); }};
};

// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Notifier.h>

namespace frc3512 {

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
    SubsystemBase() = default;
    virtual ~SubsystemBase() = default;

    /**
     * Enables the notifier
     */
    void EnablePeriodic();

    /**
     * Disables the notifier
     */
    void DisablePeriodic();

    /**
     * This function will be called asynchronously every 20 ms
     */
    virtual void SubsystemPeriodic();

private:
    frc::Notifier m_notifier{[this] { SubsystemPeriodic(); }};
};

}  // namespace frc3512

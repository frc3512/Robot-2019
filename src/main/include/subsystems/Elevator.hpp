// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>

#include "Constants.hpp"

class Elevator {
public:
    Elevator();

    void SetVelocity(double velocity);

    void ResetEncoder();

    double GetHeight();

private:
    frc::Spark m_master{kElevatorMasterID};
    frc::Spark m_slave{kElevatorSlaveID};
    frc::SpeedControllerGroup m_grbx{m_master, m_slave};

    frc::Encoder m_encoder{kEncoderA, kEncoderB};
};

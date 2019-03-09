// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/FourBarLift.hpp"

using namespace frc3512;

FourBarLift::FourBarLift() : PublishNode("FourBarLift") {
    // Set radians of encoder shaft per encoder pulse using pulses per
    // revolution
    m_encoder.SetDistancePerPulse(kFourBarDpP);
}

void FourBarLift::SetVoltage(double voltage) { m_motor.Set(voltage); }

double FourBarLift::GetDisplacement() { return m_encoder.GetDistance(); }

void FourBarLift::ResetEncoder() { m_encoder.Reset(); }

bool FourBarLift::GetTopLimit() {
    return m_topLimit.Get() == m_limitPressedState;
}

void FourBarLift::SubsystemPeriodic() {
    if (GetTopLimit()) {
        ResetEncoder();
    }
}

void FourBarLift::ProcessMessage(const HIDPacket& message) {
    SetVoltage(message.y3);
}

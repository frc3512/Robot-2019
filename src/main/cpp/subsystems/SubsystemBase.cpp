// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

using namespace frc3512;

void SubsystemBase::EnablePeriodic() { m_notifier.StartPeriodic(0.02); }

void SubsystemBase::DisablePeriodic() { m_notifier.Stop(); }

void SubsystemBase::SubsystemPeriodic() {}

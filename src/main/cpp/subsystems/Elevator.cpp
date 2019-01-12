// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include "Robot.hpp"

Elevator::Elevator() {}

void Elevator::SetVelocity(double velocity) { m_grbx.Set(velocity); }

void Elevator::ResetEncoder() { m_encoder.Reset(); }

double Elevator::GetHeight() { return m_encoder.GetDistance(); }

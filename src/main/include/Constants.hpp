// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>
#include <wpi/math>

namespace frc3512::Constants {

/*
 * Math and Conversions
 */
constexpr double kGravity = 9.80665;

namespace Robot {
constexpr int kMjpegServerPort = 1180;

/*
 * Joystick and buttons
 */

// Joystick Ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;
constexpr int kAppendageStick2Port = 3;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;
}  // namespace Robot

namespace Drivetrain {
// Motor Ports
constexpr int kLeftMasterPort = 1;
constexpr int kRightMasterPort = 7;

// Encoder Ports
constexpr int kLeftEncoderA = 2;
constexpr int kLeftEncoderB = 3;
constexpr int kRightEncoderA = 0;
constexpr int kRightEncoderB = 1;

// Controller constants
constexpr double kPositionTolerance = 0.05;  // m
constexpr double kVelocityTolerance = 2.0;   // m/s
constexpr double kAngleTolerance = 0.05;     // rad

// Physical Robot Constants
constexpr auto kWheelbaseWidth = 0.6096_m;
constexpr auto kLength = 0.9398_m;
constexpr auto kWidth = 0.990405073902434_m;
constexpr auto kWheelRadius = 0.0746125_m;
constexpr double kDriveGearRatio = 1.0 / 1.0;

// System Characterization
constexpr auto kLinearV = 3.62_V / 1_mps;
constexpr auto kLinearA = 2.5_V / 1_mps_sq;
constexpr auto kAngularV = 10.41_V / 1_rad;
constexpr auto kAngularA = 1.0_V / 1_rad_per_s / 1_s;

// Drive trapezoid profile constants
constexpr auto kMaxV = 12_V / kLinearV;  // m/s
constexpr auto kMaxA = 12_V / kLinearA;  // m/s^2

// Distance per Pulse
constexpr double kDpP = (2.0 * wpi::math::pi * kWheelRadius.to<double>()) *
                        kDriveGearRatio / 2048.0;

// Solenoid Ports
constexpr int kShifterPort = 1;
}  // namespace Drivetrain

namespace Climber {
// Motor Ports
constexpr int kDrivePort = 2;
constexpr int kLiftPort = 9;

// Climber Encoder Ports
constexpr int kLiftEncoderA = 8;
constexpr int kLiftEncoderB = 9;

// Climber Physical Constants
constexpr auto kMaxV = 0.25_mps;  // m/sec
constexpr auto kMaxA = 2.5_mps_sq;
constexpr double kGearRatio = 80 / 1;
constexpr double kSprocketRadius = 0.0323342 / 2.0;
constexpr double kDpP = (2.0 * wpi::math::pi * kSprocketRadius) / 2048.0;
constexpr double kRobotMass = 63.503;  // kg

// Setpoints
constexpr double kClimb3Height = -0.4826 - 0.0254;
constexpr double kClimb2Height = -0.1498 - 0.0254;
}  // namespace Climber

namespace FourBarLift {
// Motor Ports
constexpr int kPort = 3;

// Sensors
constexpr int kEncoderA = 6;
constexpr int kEncoderB = 7;

// Four-bar Lift Physical Constants
constexpr auto kMaxV = 1.477996_rad_per_s;
constexpr auto kMaxA = 7.782482_rad_per_s / 1_s;
constexpr double kGearRatio = 302.22 / 1;
constexpr double kMin = -1.495867;
constexpr double kMax = 0.0;
constexpr double kStallTorque = 0.71;
constexpr double kStallCurrent = 134.0;
constexpr double kLength = 0.508;  // Approx 20.0 in.
constexpr double kMass = 7.18;     // Approx 15 lbs.

// Distance per pulse (converts ticks to radians)
constexpr double kDpP = 2 * wpi::math::pi / 2048;

// Represents the height the four-bar offers when scoring in most goals
constexpr double kOffset = 0.5334;

// Setpoints
constexpr double kBottomHatch = -1.02086;
}  // namespace FourBarLift

namespace Intake {
// Motor Ports
constexpr int kLeftPort = 0;
constexpr int kRightPort = 8;

// Power Distribution Panel Ports
constexpr int kLeftPDP = 11;
constexpr int kRightPDP = 10;

// Solenoid ports
constexpr int kClawPort = 2;
}  // namespace Intake

namespace Elevator {
// Motor Ports
constexpr int kPort = 9;

// Encoder Ports
constexpr int kEncoderA = 4;
constexpr int kEncoderB = 5;

// Elevator Physical Constants
constexpr auto kMaxV = 8.6_mps;
constexpr auto kMaxA = 4.3_mps_sq;
constexpr auto kClimbMaxV = 0.25_mps;
constexpr auto kClimbMaxA = 2.5_mps_sq;
constexpr double kDrumRadius = 0.0363728 / 2.0;  // meters
constexpr double kNumMotors = 1.0;
constexpr double kStallTorque = 2.41 * kNumMotors;    // N-m
constexpr double kStallCurrent = 131.0 * kNumMotors;  // amps
constexpr double kFreeSpeed = 5330.0;                 // no load rpm
constexpr double kFreeCurrent = 2.7 * kNumMotors;     // amps
constexpr double kResistance = 12.0 / kStallCurrent;  // resistance of motor
constexpr double kKt = kStallTorque / kStallCurrent;  // torque constant
constexpr double kMin = 0.0;
constexpr double kMax = 0.8;

// Distance per Pulse
constexpr double kDpP = (2.0 * wpi::math::pi * kDrumRadius) * 2.0 / 2048.0;

// Setpoints

// Approximates the height the bottom of the elevator is off the ground
constexpr double kOffset = 0.18415;

constexpr double kFloorHeight = 0.0;

constexpr double kBottomHatch = 0.0;
constexpr double kMiddleHatch = 1.11602 - FourBarLift::kOffset - kOffset;
constexpr double kTopHatch = 1.83722 - FourBarLift::kOffset - kOffset;

constexpr double kBottomCargo = 0.0;
constexpr double kMiddleCargo = 1.33192 - FourBarLift::kOffset - kOffset;

// 0.11 compensates for the physical limitations of the elevator
constexpr double kTopCargo = 2.03312 - FourBarLift::kOffset - kOffset - 0.11;

constexpr double kCargoShip = 0.9941 - FourBarLift::kOffset - kOffset;

constexpr double kHab3 = 0.42;
constexpr double kHab2 = 0.0898;
}  // namespace Elevator

constexpr auto kDt = 0.00505_s;
constexpr int kControllerPrio = 50;
}  // namespace frc3512::Constants

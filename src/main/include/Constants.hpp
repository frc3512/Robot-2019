// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units.h>

namespace frc3512 {
namespace Constants {

/*
 * Math and Conversions
 */

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

/**
 * Converts degrees to radians
 */
template <class T>
constexpr T deg2rad(const T& value) {
    return value * kPi / 180;
}

/**
 * Converts radians to degrees
 */
template <class T>
constexpr T rad2deg(const T& value) {
    return value * 180 / kPi;
}

namespace Robot {
constexpr int kMjpegServerPort = 1180;
constexpr double kNominalVoltage = 12.0;

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
constexpr double kGearRatio = 50 / 1;
constexpr double kSprocketRadius = 0.0323342 / 2.0;
constexpr double kDpP = (2.0 * kPi * kSprocketRadius) / 2048.0;
constexpr double kRobotMass = 63.503;  // kg

// Setpoints
constexpr double kClimb3Height = -0.4826 - 0.0254;  // - 0.04;
constexpr double kClimb2Height = -0.1498 - 0.0254;
}  // namespace Climber

namespace FourBarLift {
// Motor Ports
constexpr int kPort = 3;

// Sensors
constexpr int kEncoderA = 6;
constexpr int kEncoderB = 7;

constexpr auto kMaxV = 1.25_mps;
constexpr auto kMaxA = 0.5_mps_sq;
constexpr double kGearRatio = 200 / 1;
constexpr double kMin = -1.60;
constexpr double kMax = 0.0;
constexpr double kStallTorque = 0.71;
constexpr double kStallCurrent = 134.0;
constexpr double kLength = 0.508;   // Approx 20.0 IN
constexpr double kMass = 6.803886;  // Approx 15 lbs.

// Distance per pulse (converts ticks to radians)
constexpr double kDpP = 2 * kPi / 2048 / 10;

// Represents the height the four-bar offers when scoring in most goals
constexpr double kOffset = 0.5334;

// Setpoints
constexpr double kBottomHatch = -0.883;
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
constexpr int kPort = 4;

// Encoder Ports
constexpr int kEncoderA = 4;
constexpr int kEncoderB = 5;

// Elevator Physical Constants
constexpr auto kMaxV = 2.7_mps;        // m/sec
constexpr auto kMaxA = 2.5_mps_sq;     // Reduced from 12 to please Rowe
constexpr auto kClimbMaxV = 0.25_mps;  // m/sec
constexpr auto kClimbMaxA = 2.5_mps_sq;
constexpr double kCarriageMass = 7.07;           // kilograms
constexpr double kDrumRadius = 0.0363728 / 2.0;  // meters
constexpr double kGearRatio = 45 / 12 * 7 / 1 * 40 / 40;
constexpr double kNumMotors = 2.0;
constexpr double kStallTorque = 2.41 * kNumMotors;    // N-m
constexpr double kStallCurrent = 131.0 * kNumMotors;  // amps
constexpr double kFreeSpeed = 5330.0;                 // no load rpm
constexpr double kFreeCurrent = 2.7 * kNumMotors;     // amps
constexpr double kResistance = 12.0 / kStallCurrent;  // resistance of motor
constexpr double kKt = kStallTorque / kStallCurrent;  // torque constant
constexpr double kMin = 0.0;
constexpr double kMax = 1.32;

// Distance per Pulse
constexpr double kDpP = (2.0 * kPi * kDrumRadius) * 2.0 / 2048.0;

// Setpoints

// Approximates the height the bottom of the elevator is off the ground
constexpr double kOffset = 0.18415;

constexpr double kFloorHeight = 0.0;

constexpr double kBottomHatch = 0.0;  // 0.46482;
constexpr double kMiddleHatch = 1.17602 - FourBarLift::kOffset - kOffset;
constexpr double kTopHatch = 1.88722 - FourBarLift::kOffset - kOffset;

constexpr double kBottomCargo = 0.0;
constexpr double kMiddleCargo = 1.39192 - FourBarLift::kOffset - kOffset;
// 0.06 compensates for the physical limitations of the elevator
constexpr double kTopCargo = 2.10312 - FourBarLift::kOffset - kOffset - 0.11;

constexpr double kCargoShip = 1.0541 - FourBarLift::kOffset - kOffset;

constexpr double kHab3 = 0.48;
constexpr double kHab2 = 0.1498;
}  // namespace Elevator

constexpr auto kDt_s = 0.00505_s;
}  // namespace Constants
}  // namespace frc3512

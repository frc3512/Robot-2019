// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units.h>

namespace frc3512 {

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

/*
 * Miscellaneous
 */

constexpr int kNodeQueueSize = 1024;
constexpr int kMjpegServerPort = 1180;
constexpr double kRobotVoltage = 12.0;

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

/*
 * DriveTrain
 */

// Motor Ports
constexpr int kLeftDriveMasterPort = 1;
constexpr int kRightDriveMasterPort = 7;

// Encoder Ports
constexpr int kLeftEncoderA = 2;
constexpr int kLeftEncoderB = 3;
constexpr int kRightEncoderA = 0;
constexpr int kRightEncoderB = 1;

// Solenoid Ports
constexpr int kShifterPort = 1;

/*
 * Climber
 */

// Motor Ports
constexpr int kClimberDrivePort = 2;
constexpr int kClimberLiftPort = 9;

// Climber Encoder Ports
constexpr int kLiftEncoderA = 8;
constexpr int kLiftEncoderB = 9;

/*
 * Four-bar Lift
 */

// Motor Ports
constexpr int kFourBarLiftPort = 3;

// Sensors
constexpr int kFourBarLiftEncoderA = 6;
constexpr int kFourBarLiftEncoderB = 7;

constexpr auto kFourBarLiftMaxV = 1.25_mps;
constexpr auto kFourBarLiftMaxA = 0.5_mps_sq;
constexpr double kFourBarLiftGearRatio = 200 / 1;
constexpr double kFourBarLiftMax = 0.0;
constexpr double kFourBarLiftMin = -1.60;
constexpr double kFourBarLiftStallTorque = 0.71;
constexpr double kFourBarLiftStallCurrent = 134.0;
constexpr double kFourBarLiftLength = 0.508;   // Approx 20.0 IN
constexpr double kFourBarLiftMass = 6.803886;  // Approx 15 lbs.

// Distance per pulse (converts ticks to radians)
constexpr double kFourBarLiftDpP = 2 * kPi / 2048 / 10;

// Setpoints
constexpr double kFourBarBottomHatch = -0.883;

/*
 * Intake
 */

// Motor Ports
constexpr int kLeftIntakePort = 0;
constexpr int kRightIntakePort = 8;

// Power Distribution Panel Ports
constexpr int kLeftIntakePDP = 11;
constexpr int kRightIntakePDP = 10;

// Solenoid ports
constexpr int kIntakeClawPort = 2;

// Climber Physical Constants
constexpr auto kClimberMaxV = 0.25_mps;  // m/sec
constexpr auto kClimberMaxA = 2.5_mps_sq;
constexpr double kClimberGearRatio = 50 / 1;
constexpr double kSprocketRadius = 0.0323342 / 2.0;
constexpr double kClimberDpP = (2.0 * kPi * kSprocketRadius) / 2048.0;
constexpr double kRobotMass = 63.503;  // kg

// Climber Setpoints
constexpr double kClimb3Height = -0.4826 - 0.0254;  // - 0.04;
constexpr double kClimb2Height = -0.1498 - 0.0254;

/*
 * Elevator
 */

// Motor Ports
constexpr int kElevatorPort = 4;

// Encoder Ports
constexpr int kEncoderA = 4;
constexpr int kEncoderB = 5;

// Elevator Physical Constants
constexpr auto kElevatorMaxV = 2.7_mps;     // m/sec
constexpr auto kElevatorMaxA = 2.5_mps_sq;  // Reduced from 12 to please Rowe
constexpr auto kElevatorClimbV = 0.25_mps;  // m/sec
constexpr auto kElevatorClimbA = 2.5_mps_sq;
constexpr double kCarriageMass = 7.07;           // kilograms
constexpr double kDrumRadius = 0.0363728 / 2.0;  // meters
constexpr double kElevatorGearRatio = 45 / 12 * 7 / 1 * 40 / 40;
constexpr double kNumMotors = 2.0;
constexpr double kStallTorque = 2.41 * kNumMotors;    // N-m
constexpr double kStallCurrent = 131.0 * kNumMotors;  // amps
constexpr double kFreeSpeed = 5330.0;                 // no load rpm
constexpr double kFreeCurrent = 2.7 * kNumMotors;     // amps
constexpr double kResistance = 12.0 / kStallCurrent;  // resistance of motor
constexpr double kKt = kStallTorque / kStallCurrent;  // torque constant
constexpr double kElevatorMin = 0.0;
constexpr double kElevatorMax = 1.32;

constexpr auto kDt_s = 0.00505_s;

// Distance per Pulse
constexpr double kElevatorDpP = (2.0 * kPi * kDrumRadius) * 2.0 / 2048.0;

// Setpoints

// Represents the height the four-bar offers when scoring in most goals
constexpr double kFourBarOffset = 0.5334;

// Approximates the height the bottom of the elevator is off the ground
constexpr double kElevatorOffset = 0.18415;

constexpr double kFloorHeight = 0.0;

constexpr double kBottomHatch = 0.0;  // 0.46482;
constexpr double kMiddleHatch = 1.17602 - kFourBarOffset - kElevatorOffset;
constexpr double kTopHatch = 1.88722 - kFourBarOffset - kElevatorOffset;

constexpr double kBottomCargo = 0.0;
constexpr double kMiddleCargo = 1.39192 - kFourBarOffset - kElevatorOffset;
// 0.06 compensates for the physical limitations of the elevator
constexpr double kTopCargo = 2.10312 - kFourBarOffset - kElevatorOffset - 0.11;

constexpr double kCargoShip = 1.0541 - kFourBarOffset - kElevatorOffset;

constexpr double kHab3 = 0.48;
constexpr double kHab2 = 0.1498;

}  // namespace frc3512

// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units.h>

namespace frc3512 {

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of constants:
 * > Motor IDs
 * > Solenoid Ports
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

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

// Controller constants
constexpr double kB = 1.0;
constexpr double kZeta = 1.0;
constexpr double kDt = 0.00505;
constexpr auto kDt_s = 0.00505_s;

// Drive trapezoid profile constants
constexpr auto kRobotMaxV = 5.461_mps;              // m/sec
constexpr auto kRobotMaxA = 1.0922_mps_sq;          // 3.0;           // sec
constexpr auto kRobotMaxRotateRate = 4.52_mps;      // rad/sec
constexpr auto kRobotMaxRotateAccel = 4.52_mps_sq;  // sec

// Physical Robot Constants
constexpr double kWheelbaseWidth = 0.6096;  // 24.0;
constexpr double kRobotLength = 0.9398;     // 37.0;  // Approximate
constexpr double kRobotWidth = 0.8382;      // 33.0;   // Approximate
constexpr double kWheelRadius = 0.0746125;  // 2.9375;  // 2.947
constexpr double kDriveGearRatio = 1.0 / 1.0;
constexpr double kRobotVoltage = 12.0;

// Solenoid Ports
constexpr int kShifterPort = 1;

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;

/*
 * Climber
 */

// Motor Ports
constexpr int kClimberDrivePort = 2;
constexpr int kClimberLiftPort = 9;

/*
 * Four-bar Lift
 */

// Motor Ports
constexpr int kFourBarLiftPort = 3;

// Sensors
constexpr int kFourBarLiftEncoderA = 6;
constexpr int kFourBarLiftEncoderB = 7;

constexpr auto kFourBarLiftMaxV = 0.5_mps;
constexpr auto kFourBarLiftMaxA = 2_mps_sq;
constexpr double kFourBarLiftGearRatio = 200 / 1;
constexpr double kFourBarLiftMax = 0.0;
constexpr double kFourBarLiftMin = -1.49;
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

constexpr int kLeftIntakePDP = 11;
constexpr int kRightIntakePDP = 10;

// Solenoid ports
constexpr int kIntakeClawPort = 2;

/*
 * Elevator
 */

// Motor Ports
constexpr int kElevatorPort = 4;

// Encoder Ports
constexpr int kEncoderA = 4;
constexpr int kEncoderB = 5;

// PublishNode constants
constexpr int kNodeQueueSize = 1024;

// Elevator Physical Constants
constexpr auto kElevatorMaxV = 2.7_mps;     // m/sec
constexpr auto kElevatorMaxA = 9.0_mps_sq;  // Reduced from 12 to please Rowe
constexpr double kCarriageMass = 2.0;       // kilograms
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

// Distance per Pulse for Elevator
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
// 0.07 compensates for the physical limitations of the elevator
constexpr double kTopCargo = 2.10312 - kFourBarOffset - kElevatorOffset - 0.07;

constexpr double kCargoShip = 1.0541 - kFourBarOffset - kElevatorOffset;

constexpr int kMjpegServerPort = 1180;
}  // namespace frc3512

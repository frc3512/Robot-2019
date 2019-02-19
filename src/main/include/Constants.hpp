// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

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

constexpr double kPi = 3.1415926535897932;

template <class T>
constexpr T deg2rad(const T& value) {
    return value * kPi / 180;
}

template <class T>
constexpr T rad2deg(const T& value) {
    return value * 180 / kPi;
}

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;

/*
 * DriveTrain
 */
// Motor IDs
constexpr int kLeftDriveMasterID = 1;
constexpr int kRightDriveMasterID = 7;

// Encoder IDs
constexpr int kLeftEncoderA = 2;
constexpr int kLeftEncoderB = 3;
constexpr int kRightEncoderA = 0;
constexpr int kRightEncoderB = 1;

/*
 * Climber
 */
constexpr int kClimberArmPort = 0;
constexpr int kClampForwardPort = 1;
constexpr int kClampReversePort = 2;
constexpr int kWinchID = 0;

// Solenoid Ports
constexpr int kShifterPort = 3;

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;

/*
 * Elevator
 */

// Motor IDs
constexpr int kElevatorPort = 4;

// Sensors
constexpr int kBottomLimitSwitchPort = 9;
constexpr int kTopLimitSwitchPort = 8;
constexpr int kEncoderA = 4;
constexpr int kEncoderB = 5;

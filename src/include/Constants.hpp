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

constexpr int kLeftEncoderA = 6;
constexpr int kLeftEncoderB = 5;
constexpr int kRightEncoderA = 8;
constexpr int kRightEncoderB = 7;

# FRC team 3512's 2019 robot

Source code for the 2019 comp robot: Andromeda

Source code also for the 2019 practice robot: Aether

## Setup

Install the relevant FRC toolchain for your platform
(see https://github.com/wpilibsuite/allwpilib/releases).

Install the following OS packages.

* python >= 3.6

Install the following python packages via `pip3 install --user package_name`.

* frccontrol
* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/master/wpiformat/README.rst

## Build

* `./gradlew build`

Coefficients for the state-space controllers and message parsers for the
publish-subscribe framework will be automatically generated in
`build/generated`. `build/generated/include` is specified as an include path in
build.gradle, so `#include` directives can start from that directory.

## Game

The game for 2019 is called Deep Space and this year there is no autonomous period but instead a period called "sandstorm" in which the drivers can not see into the arena but can control the robot "blindly" or via a camera, and it is up the the discretion of the teams to either use a camera or do autonomous. As well there is a standard 135 second telop period. The game is pretty plain this year in that you must put a hatch on a spaceship of three varying levels or a cargo ship with the same lower most level height of the spaceship. After you must place a ball or cargo into a scoring portal in either the cargo ship or spaceship (with heights different than the hatch placement). In order to accuire RP you can either finish the game by climbing platforms and totalling over a certain end game score and or you can place all hatches and cargo into a spaceship.


## Unique features

This years robot's unique features include:

Two PID driven systems
Four Bar
Intake and outake via wheels (similar to 2016, 2018)
Elevator system
120 Degree FOV Camera
Hall effect limit switch
Extensive Data And Driver UI Reporting

## Goals of the year

|Status|Goal|
|------|----|
|No|Pub Sub System|
|No|Data Reporting|
|No|State-space Controllers|
|No|Vision Processing|

## Roster

Mentors: Tyler Veness, William Ward

Students: Charlie Parkinson (Lead), Luke Rowe, William Jin, Kyle Quinlan, Matthew Santana, Gabriel Castellanos

# FRC team 3512's 2019 robot

Source code for the 2019 comp robot: Andromeda

Source code also for the 2019 practice robot: Aithir

## Setup

Install the relevant FRC toolchain for your platform
(see https://github.com/wpilibsuite/allwpilib/releases). Make sure
`~/frc2019/roborio/bin` is in PATH.

Install the following OS packages.

* gcc >= 6.3.0
* python >= 3.6

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/master/wpiformat/README.rst

## Build

* `./gradlew-build-athena.sh`

This runs a roboRIO build. Message parsers for the publish-subscribe framework
will be automatically generated in `build/generated`. `build/generated/include`
is specified as an include path in the Makefile, so `#include` directives can
start from that directory.

## Test

* `./gradlew-test.sh`

This runs a desktop build and executes all the unit tests in `src/test`.

## Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

## Documentation

* `./gradlew-docs.sh`

Doxygen 1.8.15 needs to be installed. The HTML documentation will be generated
in `build/docs/html` with an index.html page as the root.

## Game

The game for 2019 is called Deep Space and this year there is no autonomous period but instead a period called "sandstorm" in which the drivers can not see into the arena but can control the robot "blindly" or via a camera, and it is up the the discretion of the teams to either use a camera or do autonomous. As well there is a standard 135 second telop period. The game is pretty plain this year in that you must put a hatch on a spaceship of three varying levels or a cargo ship with the same lower most level height of the spaceship. After you must place a ball or cargo into a scoring portal in either the cargo ship or spaceship (with heights different than the hatch placement). In order to accuire RP you can either finish the game by climbing platforms and totalling over a certain end game score and or you can place all hatches and cargo into a spaceship.


## Unique features

This years robot's unique features include:

- State-space controllers
- Four Bar
- Elevator
- "Backjack" climbing mechanism
- Intake and outake via wheels (similar to 2016, 2018)
- 120 Degree FOV Camera

## Goals of the year

|Status|Goal|
|------|----|
|Yes|Pub Sub System|
|Yes|Data Reporting|
|Yes|State-space Controllers|
|No|Vision Processing|

## Roster

Mentors: Tyler Veness

Students: Charlie Parkinson (Lead), Luke Rowe, William Jin, Kyle Quinlan, Matthew Santana, Gabriel Castellanos

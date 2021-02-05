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
  * https://github.com/wpilibsuite/styleguide/blob/main/wpiformat/README.rst

## Build options

### Build everything

* `./gradlew build`

This runs a roboRIO and desktop build and runs the unit tests. Message parsers
for the publish-subscribe framework will be automatically generated in
`build/generated`. `build/generated/include` is specified as an include path in
the Makefile, so `#include` directives can start from that directory.

This may take a while, so more specific builds (see below) are recommended
instead.

### Build (athena)

* `./gradlew buildAthena`

This runs a roboRIO build.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

### Test

* `./gradlew test`

This runs a release build of the robot code's unit tests from `src/test`. They
are useful for ensuring parts of the robot code continue to work correctly after
implementing new features or refactoring existing functionality.

### Simulation GUI

* `./gradlew simulate`

This runs a debug build of the robot code in the simulation GUI.

* `./buildscripts/simulate.sh`

This runs a debug build of the robot code in the simulation GUI, but the console
output is printed to the shell instead of squelched.

### Documentation

* `./gradlew doxygen`

The source code and algorithms documentation is located in the [docs](docs)
folder. This command generates HTML documentation for the robot code from
in-source Doxygen comments. The results are placed in a `docs/html` folder with
an `index.html` page as the root.

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

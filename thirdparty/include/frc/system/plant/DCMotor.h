/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

namespace units {
UNIT_ADD(moment_of_inertia, kilogram_square_meter, kilogram_square_meters,
         kg_sq_m, compound_unit<kilogram, square_meter>)
using namespace moment_of_inertia;
}  // namespace units

namespace frc {

/**
 * Holds the constants for a DC motor.
 */
class DCMotor {
 public:
  using radians_per_second_per_volt_t =
      units::unit_t<units::compound_unit<units::radians_per_second,
                                         units::inverse<units::volt>>>;
  using newton_meters_per_ampere_t =
      units::unit_t<units::compound_unit<units::newton_meters,
                                         units::inverse<units::ampere>>>;

  units::volt_t nominalVoltage;
  units::newton_meter_t stallTorque;
  units::ampere_t stallCurrent;
  units::ampere_t freeCurrent;
  units::radians_per_second_t freeSpeed;

  // Resistance of motor
  units::ohm_t R;

  // Motor velocity constant
  radians_per_second_per_volt_t Kv;

  // Torque constant
  newton_meters_per_ampere_t Kt;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltage Voltage at which the motor constants were measured.
   * @param stallTorque    Current draw when stalled.
   * @param stallCurrent   Current draw when stalled.
   * @param freeCurrent    Current draw under no load.
   * @param freeSpeed      Angular velocity under no load.
   * @param numMotors      Number of motors in a gearbox.
   */
  DCMotor(units::volt_t nominalVoltage, units::newton_meter_t stallTorque,
          units::ampere_t stallCurrent, units::ampere_t freeCurrent,
          units::radians_per_second_t freeSpeed, int numMotors = 1);

  /**
   * Returns instance of CIM.
   */
  static DCMotor CIM(int numMotors = 1);

  /**
   * Returns instance of MiniCIM.
   */
  static DCMotor MiniCIM(int numMotors = 1);

  /**
   * Returns instance of Bag motor.
   */
  static DCMotor Bag(int numMotors = 1);

  /**
   * Returns instance of Vex 775 Pro.
   */
  static DCMotor Vex775Pro(int numMotors = 1);

  /**
   * Returns instance of Andymark RS 775-125.
   */
  static DCMotor RS775_125(int numMotors = 1);

  /**
   * Returns instance of Banebots RS 775.
   */
  static DCMotor BanebotsRS775(int numMotors = 1);

  /**
   * Returns instance of Andymark 9015.
   */
  static DCMotor Andymark9015(int numMotors = 1);

  /**
   * Returns instance of Banebots RS 550.
   */
  static DCMotor BanebotsRS550(int numMotors = 1);

  /**
   * Returns instance of NEO brushless motor.
   */
  static DCMotor NEO(int numMotors = 1);

  /**
   * Returns instance of NEO 550 brushless motor.
   */
  static DCMotor NEO550(int numMotors = 1);
};

}  // namespace frc

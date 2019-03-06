/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/system/plant/DCMotor.h"

namespace frc {

DCMotor::DCMotor(units::volt_t nominalVoltage,
                 units::newton_meter_t stallTorque,
                 units::ampere_t stallCurrent, units::ampere_t freeCurrent,
                 units::radians_per_second_t freeSpeed, int numMotors) {
  this->nominalVoltage = nominalVoltage;
  this->stallTorque = stallTorque * numMotors;
  this->stallCurrent = stallCurrent;
  this->freeCurrent = freeCurrent;
  this->freeSpeed = freeSpeed;
  this->R = nominalVoltage / stallCurrent;
  this->Kv = freeSpeed / (nominalVoltage - R * freeCurrent);
  this->Kt = this->stallTorque / stallCurrent;
}

DCMotor DCMotor::CIM(int numMotors) {
  return DCMotor(12_V, 2.42_Nm, 133_A, 2.7_A, 5310_rpm, numMotors);
}

DCMotor DCMotor::MiniCIM(int numMotors) {
  return DCMotor(12_V, 1.41_Nm, 89_A, 3_A, 5840_rpm, numMotors);
}

DCMotor DCMotor::Bag(int numMotors) {
  return DCMotor(12_V, 0.43_Nm, 53_A, 1.8_A, 13180_rpm, numMotors);
}

DCMotor DCMotor::Vex775Pro(int numMotors) {
  return DCMotor(12_V, 0.71_Nm, 134_A, 0.7_A, 18730_rpm, numMotors);
}

DCMotor DCMotor::RS775_125(int numMotors) {
  return DCMotor(12_V, 0.28_Nm, 18_A, 1.6_A, 5800_rpm, numMotors);
}

DCMotor DCMotor::BanebotsRS775(int numMotors) {
  return DCMotor(12_V, 0.72_Nm, 97_A, 2.7_A, 13050_rpm, numMotors);
}

DCMotor DCMotor::Andymark9015(int numMotors) {
  return DCMotor(12_V, 0.36_Nm, 71_A, 3.7_A, 14270_rpm, numMotors);
}

DCMotor DCMotor::BanebotsRS550(int numMotors) {
  return DCMotor(12_V, 0.38_Nm, 84_A, 0.4_A, 19000_rpm, numMotors);
}

DCMotor DCMotor::NEO(int numMotors) {
  return DCMotor(12_V, 2.6_Nm, 105_A, 1.8_A, 5676_rpm, numMotors);
}

}  // namespace frc

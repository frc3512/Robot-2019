/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/geometry/Pose2d.h"

#include <cmath>

using namespace frc;

Pose2d::Pose2d(Translation2d translation, Rotation2d rotation)
    : m_translation(translation), m_rotation(rotation) {}

Pose2d::Pose2d(units::meter_t x, units::meter_t y, Rotation2d rotation)
    : m_translation(x, y), m_rotation(rotation) {}

Pose2d Pose2d::operator+(const Transform2d& other) const {
  return TransformBy(other);
}

Pose2d& Pose2d::operator+=(const Transform2d& other) {
  m_translation += other.Translation().RotateBy(m_rotation);
  m_rotation += other.Rotation();
  return *this;
}

Pose2d Pose2d::TransformBy(const Transform2d& other) const {
  return {m_translation + (other.Translation().RotateBy(m_rotation)),
          m_rotation + other.Rotation()};
}

Pose2d Pose2d::RelativeTo(const Pose2d& other) const {
  const Transform2d transform{other, *this};
  return {transform.Translation(), transform.Rotation()};
}

Pose2d Pose2d::Exp(const Twist2d& twist) const {
  const auto dx = twist.dx;
  const auto dy = twist.dy;
  const auto dtheta = twist.dtheta.to<double>();

  const auto sinTheta = std::sin(dtheta);
  const auto cosTheta = std::cos(dtheta);

  double s, c;
  if (std::abs(dtheta) < 1E-9) {
    s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
    c = 0.5 * dtheta;
  } else {
    s = sinTheta / dtheta;
    c = (1 - cosTheta) / dtheta;
  }

  const Transform2d transform{Translation2d{dx * s - dy * c, dx * c + dy * s},
                              Rotation2d{cosTheta, sinTheta}};

  return *this + transform;
}

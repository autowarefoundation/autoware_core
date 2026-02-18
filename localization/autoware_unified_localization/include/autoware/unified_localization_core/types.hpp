// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__TYPES_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__TYPES_HPP_

#include <array>
#include <cmath>

namespace autoware::unified_localization_core
{

/** Pose with 6x6 row-major covariance (x, y, z, roll, pitch, yaw). */
struct PoseWithCovariance
{
  double timestamp_sec{0.0};
  double position_x{0.0};
  double position_y{0.0};
  double position_z{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  std::array<double, 36> covariance{};
};

/** Twist (linear + angular) with 6x6 row-major covariance. */
struct TwistWithCovariance
{
  double timestamp_sec{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
  std::array<double, 36> covariance{};
};

/** Odometry-like output (pose + twist + covariances). */
struct OdometryOutput
{
  double timestamp_sec{0.0};
  double position_x{0.0};
  double position_y{0.0};
  double position_z{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
};

/** Acceleration (linear + angular). */
struct AccelerationOutput
{
  double timestamp_sec{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

/** 3D vector (e.g. linear/angular velocity). */
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double get_yaw_from_quaternion(double ox, double oy, double oz, double ow)
{
  const double siny_cosp = 2.0 * (ow * oz + ox * oy);
  const double cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline void quaternion_from_rpy(double roll, double pitch, double yaw, double & ox, double & oy, double & oz, double & ow)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  ow = cr * cp * cy + sr * sp * sy;
  ox = sr * cp * cy - cr * sp * sy;
  oy = cr * sp * cy + sr * cp * sy;
  oz = cr * cp * sy - sr * sp * cy;
}

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__TYPES_HPP_

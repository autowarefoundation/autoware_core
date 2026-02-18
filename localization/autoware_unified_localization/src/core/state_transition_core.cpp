// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#include "autoware/unified_localization_core/state_transition_core.hpp"
#include "autoware/unified_localization_core/state_index.hpp"

#include <cmath>

namespace autoware::unified_localization_core
{

double normalize_yaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

Vector6d predict_next_state(const Vector6d & x_curr, double dt)
{
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);

  const double x = x_curr(X);
  const double y = x_curr(Y);
  const double yaw = x_curr(YAW);
  const double yaw_bias = x_curr(YAWB);
  const double vx = x_curr(VX);
  const double wz = x_curr(WZ);

  Vector6d x_next;
  x_next(X) = x + vx * std::cos(yaw + yaw_bias) * dt;
  x_next(Y) = y + vx * std::sin(yaw + yaw_bias) * dt;
  x_next(YAW) = normalize_yaw(yaw + wz * dt);
  x_next(YAWB) = yaw_bias;
  x_next(VX) = vx;
  x_next(WZ) = wz;
  return x_next;
}

Matrix6d create_state_transition_matrix(const Vector6d & x_curr, double dt)
{
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);

  const double yaw = x_curr(YAW);
  const double yaw_bias = x_curr(YAWB);
  const double vx = x_curr(VX);

  Matrix6d a = Matrix6d::Identity();
  a(X, YAW) = -vx * std::sin(yaw + yaw_bias) * dt;
  a(X, YAWB) = -vx * std::sin(yaw + yaw_bias) * dt;
  a(X, VX) = std::cos(yaw + yaw_bias) * dt;
  a(Y, YAW) = vx * std::cos(yaw + yaw_bias) * dt;
  a(Y, YAWB) = vx * std::cos(yaw + yaw_bias) * dt;
  a(Y, VX) = std::sin(yaw + yaw_bias) * dt;
  a(YAW, WZ) = dt;
  return a;
}

Matrix6d process_noise_covariance(
  double proc_cov_yaw_d, double proc_cov_vx_d, double proc_cov_wz_d)
{
  const int YAW = static_cast<int>(StateIndex::YAW);
  const int YAWB = static_cast<int>(StateIndex::YAWB);
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);

  Matrix6d q = Matrix6d::Zero();
  q(X, X) = 0.0;
  q(Y, Y) = 0.0;
  q(YAW, YAW) = proc_cov_yaw_d;
  q(YAWB, YAWB) = 0.0;
  q(VX, VX) = proc_cov_vx_d;
  q(WZ, WZ) = proc_cov_wz_d;
  return q;
}

}  // namespace autoware::unified_localization_core

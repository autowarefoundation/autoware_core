// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__MEASUREMENT_CORE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__MEASUREMENT_CORE_HPP_

#include <Eigen/Core>
#include <array>
#include <cstddef>

namespace autoware::unified_localization_core
{

Eigen::Matrix<double, 3, 6> pose_measurement_matrix();
Eigen::Matrix<double, 2, 6> twist_measurement_matrix();
Eigen::Matrix3d pose_measurement_covariance(
  const std::array<double, 36> & covariance, size_t smoothing_step);
Eigen::Matrix2d twist_measurement_covariance(
  const std::array<double, 36> & covariance, size_t smoothing_step);

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__MEASUREMENT_CORE_HPP_

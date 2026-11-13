// Copyright 2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

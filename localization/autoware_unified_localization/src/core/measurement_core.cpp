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

#include "autoware/unified_localization_core/measurement_core.hpp"

#include "autoware/unified_localization_core/covariance_index.hpp"
#include "autoware/unified_localization_core/state_index.hpp"

#include <Eigen/Core>

namespace autoware::unified_localization_core
{

Eigen::Matrix<double, 3, 6> pose_measurement_matrix()
{
  const int X = static_cast<int>(StateIndex::X);
  const int Y = static_cast<int>(StateIndex::Y);
  const int YAW = static_cast<int>(StateIndex::YAW);
  Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
  c(0, X) = 1.0;
  c(1, Y) = 1.0;
  c(2, YAW) = 1.0;
  return c;
}

Eigen::Matrix<double, 2, 6> twist_measurement_matrix()
{
  const int VX = static_cast<int>(StateIndex::VX);
  const int WZ = static_cast<int>(StateIndex::WZ);
  Eigen::Matrix<double, 2, 6> c = Eigen::Matrix<double, 2, 6>::Zero();
  c(0, VX) = 1.0;
  c(1, WZ) = 1.0;
  return c;
}

Eigen::Matrix3d pose_measurement_covariance(
  const std::array<double, 36> & covariance, size_t smoothing_step)
{
  using Idx = CovarianceIndex::Index;
  Eigen::Matrix3d r;
  r << covariance[Idx::X_X], covariance[Idx::X_Y], covariance[Idx::X_YAW], covariance[Idx::Y_X],
    covariance[Idx::Y_Y], covariance[Idx::Y_YAW], covariance[Idx::YAW_X], covariance[Idx::YAW_Y],
    covariance[Idx::YAW_YAW];
  return r * static_cast<double>(smoothing_step);
}

Eigen::Matrix2d twist_measurement_covariance(
  const std::array<double, 36> & covariance, size_t smoothing_step)
{
  // Twist covariance layout: row 0 = linear.x, row 5 = angular.z -> (0,0)=vx, (1,1)=wz
  Eigen::Matrix2d r;
  r << covariance[0], covariance[5], covariance[30], covariance[35];
  return r * static_cast<double>(smoothing_step);
}

}  // namespace autoware::unified_localization_core

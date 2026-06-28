// Copyright 2023 TIER IV, Inc.
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

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <cassert>
#include <memory>
#include <utility>
#include <vector>

// NOTE: The pure helpers (calc_weight_vec, calculate_weighted_mean_and_cov,
// estimate_xy_covariance_by_laplace_approximation, rotate_covariance_to_*, adjust_diagonal_covariance)
// live in estimate_covariance_math.cpp so they can be swapped for the Rust port via the NDT_USE_RUST
// build switch. The engine-driving estimators (estimate_xy_covariance_by_multi_ndt[_score]) are now
// templated over the NDT type in estimate_covariance.hpp (so the C++ engine and the Rust adapter both
// work). Only propose_poses_to_search stays here — it takes NdtResult, not the NDT object.

namespace pclomp
{

std::vector<Eigen::Matrix4f> propose_poses_to_search(
  const NdtResult & ndt_result, const std::vector<double> & offset_x,
  const std::vector<double> & offset_y)
{
  assert(offset_x.size() == offset_y.size());
  const Eigen::Matrix4f & center_pose = ndt_result.pose;
  const Eigen::Matrix2d rot = ndt_result.pose.topLeftCorner<2, 2>().cast<double>();
  std::vector<Eigen::Matrix4f> poses_to_search;
  for (int i = 0; i < static_cast<int>(offset_x.size()); i++) {
    const Eigen::Vector2d pose_offset(offset_x[i], offset_y[i]);
    const Eigen::Vector2d rotated_pose_offset_2d = rot * pose_offset;
    Eigen::Matrix4f curr_pose = center_pose;
    curr_pose(0, 3) += static_cast<float>(rotated_pose_offset_2d.x());
    curr_pose(1, 3) += static_cast<float>(rotated_pose_offset_2d.y());
    poses_to_search.emplace_back(curr_pose);
  }
  return poses_to_search;
}

}  // namespace pclomp

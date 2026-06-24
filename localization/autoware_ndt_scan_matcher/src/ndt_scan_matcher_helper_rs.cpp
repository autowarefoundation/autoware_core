// Copyright 2024 Autoware Foundation
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

#include "ndt_scan_matcher_helper.hpp"

#include "autoware_ndt_scan_matcher_rs.h"

#include <array>
#include <vector>

// These functions are thin adapters: the implementations live in the
// `autoware_ndt_scan_matcher_rs` Rust crate and are called over the C ABI (Phase 1 of the Rust
// port). The C++ signatures are unchanged so callers and the existing gtest are untouched.
namespace autoware::ndt_scan_matcher
{

std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation)
{
  // Serialize the rotation row-major so the Rust side is unambiguous (Eigen defaults to
  // column-major storage).
  const std::array<double, 9> rotation_row_major = {
    rotation(0, 0), rotation(0, 1), rotation(0, 2), rotation(1, 0), rotation(1, 1),
    rotation(1, 2), rotation(2, 0), rotation(2, 1), rotation(2, 2)};

  std::array<double, 36> ret_covariance{};
  autoware_ndt_scan_matcher_rs_rotate_covariance(
    src_covariance.data(), rotation_row_major.data(), ret_covariance.data());
  return ret_covariance;
}

int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  // Flatten the positions (the algorithm only uses x, y, z) into a contiguous buffer.
  std::vector<double> positions_xyz;
  positions_xyz.reserve(result_pose_msg_array.size() * 3);
  for (const auto & pose : result_pose_msg_array) {
    positions_xyz.push_back(pose.position.x);
    positions_xyz.push_back(pose.position.y);
    positions_xyz.push_back(pose.position.z);
  }

  return autoware_ndt_scan_matcher_rs_count_oscillation(
    positions_xyz.data(), result_pose_msg_array.size());
}

}  // namespace autoware::ndt_scan_matcher

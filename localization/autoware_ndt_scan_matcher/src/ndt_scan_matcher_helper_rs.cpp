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

#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <cstddef>
#include <vector>

// These functions are thin adapters: the implementations live in the
// `autoware_ndt_scan_matcher_rs` Rust crate and are called over the C ABI (Phase 1 of the Rust
// port). The C++ signatures are unchanged so callers and the existing gtest are untouched.

// Verify the geometry_msgs::msg::Pose memory layout the Rust side relies on for the zero-copy
// count_oscillation path (the Rust binding is independently checked by bindgen's layout tests).
static_assert(sizeof(geometry_msgs::msg::Pose) == 7 * sizeof(double));
static_assert(alignof(geometry_msgs::msg::Pose) == alignof(double));
static_assert(offsetof(geometry_msgs::msg::Pose, position) == 0);
static_assert(offsetof(geometry_msgs::msg::Pose, orientation) == 3 * sizeof(double));
static_assert(offsetof(geometry_msgs::msg::Point, x) == 0);
static_assert(offsetof(geometry_msgs::msg::Point, z) == 2 * sizeof(double));

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
  // Zero-copy: pass the contiguous Pose array straight to Rust, which reads positions in place.
  return autoware_ndt_scan_matcher_rs_count_oscillation(
    result_pose_msg_array.data(), result_pose_msg_array.size());
}

}  // namespace autoware::ndt_scan_matcher

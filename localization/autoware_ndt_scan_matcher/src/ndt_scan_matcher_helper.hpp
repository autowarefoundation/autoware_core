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

#ifndef NDT_SCAN_MATCHER_HELPER_HPP_
#define NDT_SCAN_MATCHER_HELPER_HPP_

#include <Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/** \brief Rotate the 3x3 position block of a 6x6 (row-major, 36-element) pose covariance by the
 * given rotation matrix, i.e. compute R * C * R^T for the upper-left 3x3 block while leaving the
 * remaining entries untouched. */
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation);

/** \brief Count the maximum number of consecutive direction inversions ("oscillations") in a
 * sequence of poses. A step is counted as an inversion when the cosine between consecutive motion
 * vectors falls below an internal threshold. */
int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);

}  // namespace autoware::ndt_scan_matcher

#endif  // NDT_SCAN_MATCHER_HELPER_HPP_

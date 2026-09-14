// Copyright 2025 The Autoware Contributors
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

#ifndef LOCALIZATION_UTIL_HPP_
#define LOCALIZATION_UTIL_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace autoware::pose_initializer
{
/// Return true when the pose timestamp is older than the allowed timeout.
///
/// The staleness is measured as the elapsed time between the message stamp and the
/// reference time `now`. A pose is considered stale when `now - stamp` exceeds `timeout`.
inline bool is_pose_stale(
  const rclcpp::Time & stamp, const rclcpp::Time & now, const double timeout)
{
  const auto elapsed = now - stamp;
  return timeout < elapsed.seconds();
}

/// Compute the 2D distance between two poses and return whether the error is small.
///
/// `error_2d` is set to the 2D Euclidean distance between `reference_pose` and `result_pose`.
/// Returns true when the error is strictly below `threshold` (i.e. the error is small),
/// and false when the error is greater than or equal to the threshold.
inline bool check_pose_error(
  const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & result_pose,
  const double threshold, double & error_2d)
{
  error_2d = autoware_utils_geometry::calc_distance2d(reference_pose, result_pose);
  return error_2d < threshold;
}
}  // namespace autoware::pose_initializer

#endif  // LOCALIZATION_UTIL_HPP_

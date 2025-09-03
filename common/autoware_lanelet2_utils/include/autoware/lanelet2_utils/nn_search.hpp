// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__LANELET2_UTILS__NN_SEARCH_HPP_
#define AUTOWARE__LANELET2_UTILS__NN_SEARCH_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <optional>
namespace autoware::experimental::lanelet2_utils
{

/**
 * @brief returns the closest lanelet to `search_pose`. If several lanelets give zero
 * distance(because `search_pose` is on those lanelets), the one that gives smallest yaw difference
 * is returned
 * @note if `search_pose` is on or within a Lanelet, the distance is zero
 */
std::optional<lanelet::ConstLanelet> get_closest_lanelet(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose);

/**
 * @brief returns the closest(in 3d distance) lanelet to `search_pose` within `dist_threshold` and
 * `yaw_threshold`
 * @note if `search_pose` is on or within a Lanelet, the distance is zero
 */
std::optional<lanelet::ConstLanelet> get_closest_lanelet_within_constraint(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__NN_SEARCH_HPP_

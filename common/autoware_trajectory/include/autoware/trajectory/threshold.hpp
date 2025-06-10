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

#ifndef AUTOWARE__TRAJECTORY__THRESHOLD_HPP_
#define AUTOWARE__TRAJECTORY__THRESHOLD_HPP_

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/Forward.h>

namespace autoware::experimental::trajectory
{

// TODO(soblin): this should be defined as one of Autoware's interface/limitations
static constexpr double k_points_minimum_dist_threshold = 0.005;

/**
 * @brief check if two base values are almost-same
 */
bool is_almost_same(const double s1, const double s2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_planning_msgs::msg::PathPoint & p1,
  const autoware_planning_msgs::msg::PathPoint & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_planning_msgs::msg::TrajectoryPoint & p1,
  const autoware_planning_msgs::msg::TrajectoryPoint & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p1,
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const lanelet::ConstPoint3d & p1, const lanelet::ConstPoint3d & p2);
}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__THRESHOLD_HPP_

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

#ifndef AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERSION_HPP_
#define AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERSION_HPP_

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::planning_topic_converter
{

using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Convert a single PathPoint to a TrajectoryPoint.
 * @details Copies the pose, longitudinal/lateral velocity and heading rate fields. The
 * TrajectoryPoint fields that have no PathPoint counterpart (acceleration_mps2,
 * front_wheel_angle_rad, rear_wheel_angle_rad and time_from_start) are left at their
 * default-constructed values. PathPoint::is_final is not propagated.
 * @param point input path point.
 * @return the corresponding trajectory point.
 */
TrajectoryPoint convertToTrajectoryPoint(const PathPoint & point);

/**
 * @brief Convert a sequence of PathPoint to a sequence of TrajectoryPoint.
 * @details Applies convertToTrajectoryPoint() element-wise, preserving order and size.
 * @param points input path points.
 * @return the corresponding trajectory points.
 */
std::vector<TrajectoryPoint> convertToTrajectoryPoints(const std::vector<PathPoint> & points);

}  // namespace autoware::planning_topic_converter

#endif  // AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERSION_HPP_

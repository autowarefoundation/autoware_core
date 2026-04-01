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

#ifndef AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/path_point.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/trajectory_point.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <optional>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @brief Build a trajectory with the default cubic or optional akima setting.
 * @param[in] points input points
 * @param[in] use_akima if true, use akima spline instead
 * @return return nullopt only if trajectory build fails
 */
template <typename PointType>
std::optional<Trajectory<PointType>> pretty_build(
  const std::vector<PointType> & points, const bool use_akima = false)
{
  using Builder = typename Trajectory<PointType>::Builder;

  if (use_akima) {
    const auto try_trajectory =
      Builder{}.template set_xy_interpolator<interpolator::AkimaSpline>().build(points);
    if (!try_trajectory) {
      return std::nullopt;
    }
    return try_trajectory.value();
  }

  const auto try_trajectory = Builder{}.build(points);
  if (!try_trajectory) {
    return std::nullopt;
  }
  return try_trajectory.value();
}

extern template std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
pretty_build(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool use_akima);

extern template std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool use_akima);

extern template std::optional<Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>>
pretty_build(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const bool use_akima);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__PRETTY_BUILD_HPP_

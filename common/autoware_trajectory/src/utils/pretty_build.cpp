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

#include "autoware/trajectory/utils/pretty_build.hpp"

#include <string>
#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail
{

tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate3(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 =
    get_geometry_msgs_pose<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 =
    get_geometry_msgs_pose<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position = mid_position;
  point.point.pose.orientation = mid_quat;
  point.point.longitudinal_velocity_mps = inputs.at(0).point.longitudinal_velocity_mps;
  point.point.lateral_velocity_mps = inputs.at(0).point.lateral_velocity_mps;
  point.point.heading_rate_rps = inputs.at(0).point.heading_rate_rps;
  point.lane_ids = inputs.at(0).lane_ids;

  return std::vector{inputs[0], point, inputs[1]};
}

tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 = get_geometry_msgs_pose<autoware_planning_msgs::msg::PathPoint>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 = get_geometry_msgs_pose<autoware_planning_msgs::msg::PathPoint>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position = mid_position;
  point.pose.orientation = mid_quat;
  point.longitudinal_velocity_mps = inputs.at(0).longitudinal_velocity_mps;
  point.lateral_velocity_mps = inputs.at(0).lateral_velocity_mps;
  point.heading_rate_rps = inputs.at(0).heading_rate_rps;

  return std::vector{inputs[0], point, inputs[1]};
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 =
    get_geometry_msgs_pose<autoware_planning_msgs::msg::TrajectoryPoint>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 =
    get_geometry_msgs_pose<autoware_planning_msgs::msg::TrajectoryPoint>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position = mid_position;
  point.pose.orientation = mid_quat;
  point.longitudinal_velocity_mps = inputs.at(0).longitudinal_velocity_mps;
  point.lateral_velocity_mps = inputs.at(0).lateral_velocity_mps;
  point.heading_rate_rps = inputs.at(0).heading_rate_rps;
  point.acceleration_mps2 = inputs.at(0).acceleration_mps2;
  point.front_wheel_angle_rad = inputs.at(0).front_wheel_angle_rad;
  point.rear_wheel_angle_rad = inputs.at(0).rear_wheel_angle_rad;

  return std::vector{inputs[0], point, inputs[1]};
}

//
template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate4(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate4(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate4(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);

//
template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate5(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate5(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate5(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);
}  // namespace detail

//
template std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
pretty_build(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool use_akima = false);

template std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool use_akima = false);
template std::optional<Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const bool use_akima = false);

}  // namespace autoware::experimental::trajectory

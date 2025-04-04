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

#include "autoware/trajectory/utils/populate.hpp"

#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/interpolator/spherical_linear.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <string>
#include <vector>

namespace autoware::trajectory
{

tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate3(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected("cannot populate3() from #" + std::to_string(inputs.size()) + " points!");
  }

  // assert(inputs.size() == 2);

  const auto & p1 = inputs.at(0).point.pose;
  const auto & pos1 = p1.position;
  const auto & p2 = inputs.at(1).point.pose;
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

tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate4(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs)
{
  if (inputs.size() >= 4) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected("cannot populate4() from #" + std::to_string(inputs.size()) + " points!");
  }

  const auto try_inputs3 = populate3(inputs);
  if (!try_inputs3) {
    return tl::unexpected(try_inputs3.error());
  }
  const auto & inputs3 = inputs.size() == 2 ? try_inputs3.value() : inputs;

  const auto input3_interpolation_result =
    Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>::Builder{}
      .set_xy_interpolator<interpolator::Linear>()
      .set_z_interpolator<interpolator::Linear>()
      .set_orientation_interpolator<interpolator::SphericalLinear>()
      .build(inputs3);

  // LCOV_EXCL_START
  if (!input3_interpolation_result) {
    // actually this block is impossible because 3 points are given, which is sufficient for Linear
    // interpolation
    return tl::unexpected(std::string("failed Linear interpolation in populate4()!"));
  }
  // LCOV_EXCL_END

  const auto & interpolation = input3_interpolation_result.value();

  const auto bases = interpolation.get_internal_bases();
  // assert(bases.size() == 3);
  const auto bases_diff =
    std::vector<double>{(bases.at(1) - bases.at(0)), (bases.at(2) - bases.at(1))};
  if (bases_diff.at(0) >= bases_diff.at(1)) {
    const auto new_base = (bases.at(0) + bases.at(1)) / 2.0;
    return std::vector{
      inputs3.at(0), interpolation.compute(new_base), inputs3.at(1), inputs3.at(2)};
  }
  const auto new_base = (bases.at(1) + bases.at(2)) / 2.0;
  return std::vector{inputs3.at(0), inputs3.at(1), interpolation.compute(new_base), inputs3.at(2)};
}

}  // namespace autoware::trajectory

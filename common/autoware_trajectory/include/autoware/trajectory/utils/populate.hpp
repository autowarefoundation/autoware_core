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

#ifndef AUTOWARE__TRAJECTORY__UTILS__POPULATE_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__POPULATE_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory
{

/**
 * @brief if the input point size is less than 3, add 3rd point, otherwise return as is
 * @param[in] inputs vector of point whose size is at least 2
 * @return the vector of points whose size is at least 3, or error reason
 * @note {x, y, z}/orientation are interpolated by Linear/SphericalLinear. other properties as
 * interpolated by StairStep
 */
tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate3(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

/**
 * @brief if the input point size is less than 4, add 4th point, otherwise return as is
 * @param[in] inputs inputs
 * @return optional the vector of points whose size is at least 4, so that it can be interpolated by
 * CubicSpline, or error reason
 * @note {x, y, z}/orientation are interpolated by Linear/SphericalLinear. other properties as
 * interpolated by StairStep
 */
tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate4(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

/**
 * @brief if the input point size is less than 5, add 5th point, otherwise return as is
 * @param[in] inputs inputs
 * @return the vector of points whose size is at least 5, so that it can be interpolated by
 * AkimaSpline, or error reason
 * @note {x, y, z}/orientation are interpolated by Cubic/SphericalLinear. other properties as
 * interpolated by StairStep
std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> populate5(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs,
  const size_t minimum_number_of_points);
 */

}  // namespace autoware::trajectory
#endif  // AUTOWARE__TRAJECTORY__UTILS__POPULATE_HPP_

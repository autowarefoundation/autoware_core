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

#ifndef AUTOWARE__LANELET2_UTILS__STOP_LINE_HPP_
#define AUTOWARE__LANELET2_UTILS__STOP_LINE_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <lanelet2_core/Forward.h>

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{
/**
 * @brief Get stop line from road marking regulatory element from a lanelet, optionally checking
 * matching ID.
 *
 * @param[in] lanelet input lanelet
 * @return Return either first stop line or std::nullopt if no stop line exists
 */

std::optional<lanelet::ConstLineString3d> get_stop_line_from_deprecated_crosswalk(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from a no stopping area regulatory element within a lanelet
 *
 * @param[in] lanelet input lanelet
 * @return Return either first stop line or std::nullopt if no stop line exists
 */
std::optional<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from a detection area regulatory element within a lanelet
 *
 * @param[in] lanelet input lanelet
 * @return Return either first stop line or std::nullopt if no stop line exists
 */
std::optional<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from an intersection marking regulatory element within a lanelet
 *
 * @param[in] lanelet input lanelet
 * @return Return either first stop line or std::nullopt if no stop line exists
 */
std::optional<lanelet::ConstLineString3d> get_stop_line_from_intersection_marking(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from a stop sign regulatory element within a lanelet
 *
 * @param[in] lanelet input lanelet
 * @return Return either first stop line or std::nullopt if no stop line exists
 */
std::optional<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelet & lanelet);
}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__STOP_LINE_HPP_

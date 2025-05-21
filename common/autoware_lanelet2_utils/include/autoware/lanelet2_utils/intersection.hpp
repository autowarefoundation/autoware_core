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

#ifndef AUTOWARE__LANELET2_UTILS__INTERSECTION_HPP_
#define AUTOWARE__LANELET2_UTILS__INTERSECTION_HPP_

#include <lanelet2_core/Forward.h>

#include <optional>

namespace autoware::experimental::lanelet2_utils
{

static constexpr const char * k_turn_direction = "turn_direction";
static constexpr const char * k_turn_direction_straight = "straight";
static constexpr const char * k_turn_direction_left = "left";
static constexpr const char * k_turn_direction_right = "right";

enum TurnDirection : int { Straight = 0, Left, Right };

/**
 * @brief check if given lanelet has "turn_direction" attribute
 * @param [in] input lanelet
 * @return true if and only if the given lanelet has "turn_direction" attribute
 */
bool is_intersection_lanelet(const lanelet::ConstLanelet & lanelet);

/**
 * @brief check if given lanelet has "turn_direction" attribute and the value is "straight"
 * @param [in] input lanelet
 * @return true if and only if the given lanelet has "turn_direction" attribute and the value is
 * "straight"
 */
bool is_straight_direction(const lanelet::ConstLanelet & lanelet);

/**
 * @brief check if given lanelet has "turn_direction" attribute and the value is "left"
 * @param [in] input lanelet
 * @return true if and only if the given lanelet has "turn_direction" attribute and the value is
 * "left"
 */
bool is_left_direction(const lanelet::ConstLanelet & lanelet);

/**
 * @brief check if given lanelet has "turn_direction" attribute and the value is "right"
 * @param [in] input lanelet
 * @return true if and only if the given lanelet has "turn_direction" attribute and the value is
 * "right"
 */
bool is_right_direction(const lanelet::ConstLanelet & lanelet);

/**
 * @brief get the turn_direction value
 * @param [in] input lanelet
 * @return valid TurnDirection value if `lanelet` has valid "turn_direction" value, otherwise null
 * "right"
 */
std::optional<TurnDirection> get_turn_direction(const lanelet::ConstLanelet & lanelet);

}  // namespace autoware::experimental::lanelet2_utils
#endif  // AUTOWARE__LANELET2_UTILS__INTERSECTION_HPP_

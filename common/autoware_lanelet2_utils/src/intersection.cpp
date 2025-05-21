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

#include <autoware/lanelet2_utils/intersection.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <cstring>
namespace autoware::experimental::lanelet2_utils
{

bool is_intersection_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.hasAttribute(k_turn_direction);
}

bool is_straight_direction(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(lanelet.attributeOr(k_turn_direction, "else"), k_turn_direction_straight) == 0;
}

bool is_left_direction(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(lanelet.attributeOr(k_turn_direction, "else"), k_turn_direction_left) == 0;
}

bool is_right_direction(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(lanelet.attributeOr(k_turn_direction, "else"), k_turn_direction_right) == 0;
}

std::optional<TurnDirection> get_turn_direction(const lanelet::ConstLanelet & lanelet)
{
  if (is_straight_direction(lanelet)) {
    return TurnDirection::Straight;
  }
  if (is_left_direction(lanelet)) {
    return TurnDirection::Left;
  }
  if (is_right_direction(lanelet)) {
    return TurnDirection::Right;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::lanelet2_utils

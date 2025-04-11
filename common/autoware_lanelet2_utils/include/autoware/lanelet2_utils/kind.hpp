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

#ifndef AUTOWARE__LANELET2_UTILS__KIND_HPP_
#define AUTOWARE__LANELET2_UTILS__KIND_HPP_

#include <lanelet2_core/Forward.h>

namespace autoware::experimental::lanelet2_utils
{
static constexpr const char * k_road_lane_type = "road";
static constexpr const char * k_shoulder_lane_type = "road_shoulder";
static constexpr const char * k_bicycle_lane_type = "bicycle_lane";

/*
 * TODO(soblin): distinguish road lane type
class RoadLane : public lanelet::ConstLanelet
{
};

class RoadShoulderLane : public lanelet::ConstLanelet
{
};

class BicycleLane : public lanelet::ConstLanelet
{
};
*/

/**
 * @brief check if the given lanelet type has "road" subtype
 * @param [in] lanelet input lanelet
 * @return if the lanelet is road or not
 */
bool is_road_lane(const lanelet::ConstLanelet & lanelet);

/**
 * @brief check if the given lanelet type has "road_shoulder" subtype
 * @param [in] lanelet input lanelet
 * @return if the lanelet is road_shoulder or not
 */
bool is_shoulder_lane(const lanelet::ConstLanelet & lanelet);

/**
 * @brief check if the given lanelet type has "bicycle_lane" subtype
 * @param [in] lanelet input lanelet
 * @return if the lanelet is bicycle_lane or not
 */
bool is_bicycle_lane(const lanelet::ConstLanelet & lanelet);
}  // namespace autoware::experimental::lanelet2_utils
#endif  // AUTOWARE__LANELET2_UTILS__KIND_HPP_

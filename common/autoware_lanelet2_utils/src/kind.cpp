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

#include <autoware/lanelet2_utils/kind.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <cstring>
namespace autoware::experimental::lanelet2_utils
{
bool is_road_lane(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(
           lanelet.attributeOr(lanelet::AttributeName::Subtype, "none"), k_road_lane_type) == 0;
}

bool is_shoulder_lane(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(
           lanelet.attributeOr(lanelet::AttributeName::Subtype, "none"), k_shoulder_lane_type) == 0;
}

bool is_bicycle_lane(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(
           lanelet.attributeOr(lanelet::AttributeName::Subtype, "none"), k_bicycle_lane_type) == 0;
}
}  // namespace autoware::experimental::lanelet2_utils

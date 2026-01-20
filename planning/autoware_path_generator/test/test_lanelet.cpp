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

#include "utils_test.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::path_generator
{
TEST_F(UtilsTest, getPreviousLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = get_lanelet_from_id(50);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, *route_manager_);

    ASSERT_TRUE(prev_lanelet.has_value());
    ASSERT_EQ(prev_lanelet->id(), 125);
  }

  {  // The given lanelet is at the start of the route
    const auto lanelet = get_lanelet_from_id(10323);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, *route_manager_);

    ASSERT_FALSE(prev_lanelet.has_value());
  }
}

TEST_F(UtilsTest, getNextLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = get_lanelet_from_id(50);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, *route_manager_);

    ASSERT_TRUE(next_lanelet.has_value());
    ASSERT_EQ(next_lanelet->id(), 122);
  }

  {  // The given lanelet is at the end of the route
    const auto lanelet = get_lanelet_from_id(122);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, *route_manager_);

    ASSERT_FALSE(next_lanelet.has_value());
  }
}
}  // namespace autoware::path_generator

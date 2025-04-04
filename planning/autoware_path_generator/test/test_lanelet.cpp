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

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::path_generator
{
TEST_F(UtilsTest, getPreviousLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(50);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, planner_data_);

    ASSERT_TRUE(prev_lanelet.has_value());
    ASSERT_EQ(prev_lanelet->id(), 125);
  }

  {  // The given lanelet is at the start of the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(10323);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, planner_data_);

    ASSERT_FALSE(prev_lanelet.has_value());
  }
}

TEST_F(UtilsTest, getNextLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(50);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, planner_data_);

    ASSERT_TRUE(next_lanelet.has_value());
    ASSERT_EQ(next_lanelet->id(), 122);
  }

  {  // The given lanelet is at the end of the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(122);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, planner_data_);

    ASSERT_FALSE(next_lanelet.has_value());
  }
}

TEST_F(UtilsTest, getLaneletsWithinRouteUpTo)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(50);
    const auto distance = 30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_EQ(prev_lanelets->size(), 1);
    ASSERT_EQ(prev_lanelets->at(0).id(), 125);
  }

  {  // The given distance exceeds the route section
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(50);
    const auto distance = 100.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_EQ(prev_lanelets->size(), 2);
    ASSERT_EQ(prev_lanelets->at(0).id(), 10323);
    ASSERT_EQ(prev_lanelets->at(1).id(), 125);
  }

  {  // The given distance is negative
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(50);
    const auto distance = -30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_TRUE(prev_lanelets->empty());
  }

  {  // The given lanelet is not within the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(10257);
    const auto distance = 30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_FALSE(prev_lanelets.has_value());
  }
}

TEST_F(UtilsTest, getLaneletsWithinRouteAfter)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(125);
    const auto distance = 30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_EQ(next_lanelets->size(), 2);
    ASSERT_EQ(next_lanelets->at(0).id(), 50);
    ASSERT_EQ(next_lanelets->at(1).id(), 122);
  }

  {  // The given distance exceeds the route section
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(125);
    const auto distance = 100.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_EQ(next_lanelets->size(), 2);
    ASSERT_EQ(next_lanelets->at(0).id(), 50);
    ASSERT_EQ(next_lanelets->at(1).id(), 122);
  }

  {  // The given distance is negative
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(125);
    const auto distance = -30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_TRUE(next_lanelets->empty());
  }

  {  // The given lanelet is not within the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(10257);
    const auto distance = 30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_FALSE(next_lanelets.has_value());
  }
}
}  // namespace autoware::path_generator

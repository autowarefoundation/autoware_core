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

#ifndef UTILS_TEST_HPP_
#define UTILS_TEST_HPP_

#include "autoware/path_generator/utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace autoware::path_generator
{
class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle_info_ = vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);

    set_map("autoware_test_utils", "lanelet2_map.osm");
    set_route("autoware_path_generator", "common_route.yaml");
    set_path("autoware_path_generator", "common_path.yaml");
  }

  void set_map(const std::string & package_name, const std::string & map_filename)
  {
    const auto lanelet_map_path =
      autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_filename);
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet_map_path);
    if (map_bin_msg.header.frame_id == "") {
      throw std::runtime_error(
        "Frame ID of the map is empty. The file might not exist or be corrupted:" +
        lanelet_map_path);
    }

    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      map_bin_msg, planner_data_.lanelet_map_ptr, &planner_data_.traffic_rules_ptr,
      &planner_data_.routing_graph_ptr);
  }

  void set_route(const std::string & package_name, const std::string & route_filename)
  {
    if (!planner_data_.lanelet_map_ptr) {
      throw std::runtime_error("Map not set");
    }

    const auto route_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, route_filename);
    const auto route =
      autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
        route_path);
    if (!route) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
    }

    planner_data_.route_frame_id = route->header.frame_id;
    planner_data_.goal_pose = route->goal_pose;

    planner_data_.route_lanelets.clear();
    planner_data_.preferred_lanelets.clear();
    planner_data_.start_lanelets.clear();
    planner_data_.goal_lanelets.clear();

    size_t primitives_num = 0;
    for (const auto & route_section : route->segments) {
      primitives_num += route_section.primitives.size();
    }
    planner_data_.route_lanelets.reserve(primitives_num);

    for (const auto & route_section : route->segments) {
      for (const auto & primitive : route_section.primitives) {
        const auto id = primitive.id;
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
        planner_data_.route_lanelets.push_back(lanelet);
        if (id == route_section.preferred_primitive.id) {
          planner_data_.preferred_lanelets.push_back(lanelet);
        }
      }
    }

    const auto set_lanelets_from_segment =
      [&](
        const autoware_planning_msgs::msg::LaneletSegment & segment,
        lanelet::ConstLanelets & lanelets) {
        lanelets.reserve(segment.primitives.size());
        for (const auto & primitive : segment.primitives) {
          const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
          lanelets.push_back(lanelet);
        }
      };
    set_lanelets_from_segment(route->segments.front(), planner_data_.start_lanelets);
    set_lanelets_from_segment(route->segments.back(), planner_data_.goal_lanelets);
  }

  void set_path(const std::string & package_name, const std::string & path_filename)
  {
    const auto path_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, path_filename);
    try {
      path_ = autoware::test_utils::parse<
                std::optional<autoware_internal_planning_msgs::msg::PathWithLaneId>>(path_path)
                .value();
    } catch (const std::exception &) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + path_path + ". The file might be corrupted.");
    }
  }

  lanelet::ConstLanelet get_lanelet_closest_to_pose(const geometry_msgs::msg::Pose & pose) const
  {
    lanelet::ConstLanelet lanelet;
    if (!lanelet::utils::query::getClosestLanelet(
          planner_data_.preferred_lanelets, pose, &lanelet)) {
      throw std::runtime_error("Failed to get the closest lanelet to the given point.");
    }
    return lanelet;
  }

  lanelet::ConstLanelets get_lanelets_from_ids(const lanelet::Ids & ids) const
  {
    if (!planner_data_.lanelet_map_ptr) {
      throw std::runtime_error("Map not set");
    }

    lanelet::ConstLanelets lanelets;
    for (const auto & id : ids) {
      lanelets.push_back(planner_data_.lanelet_map_ptr->laneletLayer.get(id));
    }
    return lanelets;
  }

  vehicle_info_utils::VehicleInfo vehicle_info_;
  PlannerData planner_data_;
  autoware_internal_planning_msgs::msg::PathWithLaneId path_;
};
}  // namespace autoware::path_generator

#endif  // UTILS_TEST_HPP_

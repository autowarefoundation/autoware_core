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

#include "../src/utils.hpp"
#include "autoware/path_generator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <memory>
#include <string>

namespace autoware::path_generator
{
using experimental::lanelet2_utils::to_ros;

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
    lanelet_map_bin_ = std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(
      autoware::test_utils::make_map_bin_msg(lanelet_map_path));
    if (lanelet_map_bin_->header.frame_id == "") {
      throw std::runtime_error(
        "Frame ID of the map is empty. The file might not exist or be corrupted:" +
        lanelet_map_path);
    }

    route_manager_.reset();
  }

  void set_route(const std::string & package_name, const std::string & route_filename)
  {
    if (!lanelet_map_bin_) {
      throw std::runtime_error("Map not set");
    }

    const auto route_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, route_filename);
    route_ = autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
      route_path);
    if (!route_) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
    }

    route_manager_ = experimental::lanelet2_utils::RouteManager::create(
      *lanelet_map_bin_, *route_, geometry_msgs::msg::Pose{});
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
    const auto closest_lanelet = route_manager_->get_closest_preferred_route_lanelet(pose);
    if (!closest_lanelet) {
      throw std::runtime_error("Failed to get the closest lanelet to the given pose.");
    }
    return *closest_lanelet;
  }

  lanelet::ConstLanelet get_lanelet_from_id(const lanelet::Id id) const
  {
    return route_manager_->lanelet_map_ptr()->laneletLayer.get(id);
  }

  lanelet::ConstLanelets get_lanelets_from_ids(const lanelet::Ids & ids) const
  {
    lanelet::ConstLanelets lanelets(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      lanelets[i] = get_lanelet_from_id(ids[i]);
    }
    return lanelets;
  }

  vehicle_info_utils::VehicleInfo vehicle_info_;
  std::optional<experimental::lanelet2_utils::RouteManager> route_manager_{std::nullopt};

  autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr lanelet_map_bin_{nullptr};
  std::optional<autoware_planning_msgs::msg::LaneletRoute> route_{std::nullopt};
  autoware_internal_planning_msgs::msg::PathWithLaneId path_;
};
}  // namespace autoware::path_generator

#endif  // UTILS_TEST_HPP_

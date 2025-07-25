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

#include "autoware/path_generator/node.hpp"

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
namespace
{
PathGenerator::InputData create_input_data()
{
  PathGenerator::InputData input_data;

  const auto lanelet_map_path =
    ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils") +
    "/sample_map/dense_centerline/lanelet2_map.osm";
  const auto lanelet_map_bin = autoware::test_utils::make_map_bin_msg(lanelet_map_path);
  if (lanelet_map_bin.header.frame_id == "") {
    throw std::runtime_error(
      "Frame ID of the map is empty. The file might not exist or be corrupted:" + lanelet_map_path);
  }
  input_data.lanelet_map_bin_ptr = std::make_shared<LaneletMapBin>(lanelet_map_bin);

  const auto route_path = autoware::test_utils::get_absolute_path_to_route(
    "autoware_path_generator", "dense_centerline_route.yaml");
  const auto route =
    autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
      route_path);
  if (!route) {
    throw std::runtime_error(
      "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
  }
  input_data.route_ptr = std::make_shared<LaneletRoute>(*route);

  return input_data;
}
}  // namespace

TEST(DenseCenterlineTest, generatePath)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto path_generator_dir =
    ament_index_cpp::get_package_share_directory("autoware_path_generator");

  const auto node_options = rclcpp::NodeOptions{}.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     autoware_test_utils_dir + "/config/test_nearest_search.param.yaml", "--params-file",
     path_generator_dir + "/config/path_generator.param.yaml"});

  PathGenerator path_generator(node_options);

  const auto input_data = create_input_data();
  path_generator.set_planner_data(input_data);

  Params params;
  path_generator.get_parameter("path_length.backward", params.path_length.backward);
  path_generator.get_parameter("path_length.forward", params.path_length.forward);
  path_generator.get_parameter(
    "waypoint.connection_gradient_from_centerline",
    params.waypoint.connection_gradient_from_centerline);
  path_generator.get_parameter(
    "goal_connection.connection_section_length", params.goal_connection.connection_section_length);
  path_generator.get_parameter(
    "smooth_goal_connection.pre_goal_offset", params.goal_connection.pre_goal_offset);

  const auto path = path_generator.generate_path(input_data.route_ptr->start_pose, params);

  ASSERT_TRUE(path.has_value());
  ASSERT_FALSE(path->points.empty());
}
}  // namespace autoware::path_generator

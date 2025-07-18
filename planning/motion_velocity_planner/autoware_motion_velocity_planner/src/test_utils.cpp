// Copyright 2024 TIER IV, Inc.
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

#include "autoware/motion_velocity_planner/test_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: motion_velocity_planner → test_node_
  test_manager->subscribeOutput<autoware_planning_msgs::msg::Trajectory>(
    "motion_velocity_planner/output/trajectory");

  return test_manager;
}

std::shared_ptr<MotionVelocityPlannerNode> generateNode(
  const std::vector<PluginInfo> & plugin_info_vec)
{
  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto motion_velocity_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner");

  const auto get_motion_velocity_module_config = [](const std::string & module) {
    const auto package_name = "autoware_motion_velocity_" + module + "_module";
    const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
    return package_path + "/config/" + module + ".param.yaml";
  };

  std::vector<std::string> plugin_names;
  for (const auto & plugin_info : plugin_info_vec) {
    plugin_names.emplace_back(plugin_info.plugin_name);
  }

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("launch_modules", plugin_names);
  params.emplace_back("is_simulation", false);
  node_options.parameter_overrides(params);

  auto yaml_files = std::vector<std::string>{
    autoware_test_utils_dir + "/config/test_common.param.yaml",
    autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
    motion_velocity_planner_dir + "/config/motion_velocity_planner.param.yaml"};
  for (const auto & plugin_info : plugin_info_vec) {
    yaml_files.push_back(get_motion_velocity_module_config(plugin_info.module_name));
  }

  autoware::test_utils::updateNodeOptions(node_options, yaml_files);

  // TODO(Takagi, Isamu): set launch_modules

  return std::make_shared<MotionVelocityPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<MotionVelocityPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "/tf", autoware::test_utils::makeTFMsg(test_target_node, "base_link", "map"));
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/trajectory",
    autoware_planning_msgs::msg::Trajectory{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/accel",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/dynamic_objects",
    autoware_perception_msgs::msg::PredictedObjects{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/no_ground_pointcloud",
    sensor_msgs::msg::PointCloud2{}.set__header(
      std_msgs::msg::Header{}.set__frame_id("base_link")));
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/occupancy_grid",
    autoware::test_utils::makeCostMapMsg());
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner/input/traffic_signals",
    autoware_perception_msgs::msg::TrafficLightGroupArray{});
}
}  // namespace autoware::motion_velocity_planner

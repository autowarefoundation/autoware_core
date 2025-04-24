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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto test_manager =
    std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto path_generator_dir =
    ament_index_cpp::get_package_share_directory("autoware_path_generator");

  const auto node_options = rclcpp::NodeOptions{}.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     autoware_test_utils_dir + "/config/test_nearest_search.param.yaml", "--params-file",
     path_generator_dir + "/config/path_generator.param.yaml"});

  auto test_target_node = std::make_shared<autoware::path_generator::PathGenerator>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "path_generator/input/vector_map", autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    test_target_node, "path_generator/input/odometry",
    autoware::test_utils::makeOdometry().set__pose(
      geometry_msgs::msg::PoseWithCovariance{}.set__pose(
        autoware::test_utils::makeBehaviorNormalRoute().start_pose)));

  // create subscriber in test_manager
  test_manager->subscribeOutput<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "path_generator/output/path");

  const std::string route_topic_name = "path_generator/input/route";

  // test with normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, route_topic_name));

  // test with the goal on left side
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorGoalOnLeftSide(test_target_node, route_topic_name));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalRoute(test_target_node, route_topic_name));

  rclcpp::shutdown();
}

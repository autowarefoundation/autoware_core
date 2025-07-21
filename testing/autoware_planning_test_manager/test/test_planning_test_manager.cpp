// Copyright 2025 Tier IV, Inc.
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

#include "autoware/planning_test_manager/autoware_planning_test_manager.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

TEST(PlanningTestManager, CommunicationTest)
{
  using autoware_internal_planning_msgs::msg::PathWithLaneId;
  using autoware_planning_msgs::msg::LaneletRoute;
  using autoware_planning_msgs::msg::Path;
  using autoware_planning_msgs::msg::Trajectory;
  using nav_msgs::msg::Odometry;

  rclcpp::init(0, nullptr);

  // instantiate test_manager with PlanningInterfaceTestManager type
  auto test_manager =
    std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  // instantiate the TargetNode for test
  auto test_target_node = std::make_shared<rclcpp::Node>("target_node_for_communication_test");

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Trajectory>("normal_trajectory_for_test");
  test_manager->testWithNormalTrajectory(test_target_node, "normal_trajectory_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Trajectory>("abnormal_trajectory_for_test");
  test_manager->testWithAbnormalTrajectory(test_target_node, "abnormal_trajectory_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<LaneletRoute>("normal_route_for_test");
  test_manager->testWithNormalRoute(test_target_node, "normal_route_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<LaneletRoute>("abnormal_route_for_test");
  test_manager->testWithAbnormalRoute(test_target_node, "abnormal_route_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<LaneletRoute>("behavior_normal_route_for_test");
  test_manager->testWithBehaviorNormalRoute(test_target_node, "behavior_normal_route_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<LaneletRoute>("behavior_goal_on_left_side_route_for_test");
  test_manager->testWithBehaviorGoalOnLeftSide(
    test_target_node, "behavior_goal_on_left_side_route_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<PathWithLaneId>("normal_path_with_lane_id_for_test");
  test_manager->testWithNormalPathWithLaneId(test_target_node, "normal_path_with_lane_id_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<PathWithLaneId>("abnormal_path_with_lane_id_for_test");
  test_manager->testWithAbnormalPathWithLaneId(
    test_target_node, "abnormal_path_with_lane_id_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Path>("normal_path_for_test");
  test_manager->testWithNormalPath(test_target_node, "normal_path_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Path>("abnormal_path_for_test");
  test_manager->testWithAbnormalPath(test_target_node, "abnormal_path_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Odometry>("off_track_initial_poses_for_test");
  test_manager->testWithOffTrackInitialPoses(test_target_node, "off_track_initial_poses_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  test_manager->resetReceivedTopicNum();
  test_manager->subscribeOutput<Odometry>("off_track_odometry_for_test");
  test_manager->testWithOffTrackOdometry(test_target_node, "off_track_odometry_for_test");
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // shutdown ROS context
  rclcpp::shutdown();
}

TEST(PlanningTestManager, CallbackSubscriptionTest)
{
  using nav_msgs::msg::Odometry;
  const std::string off_track_odometry_topic = "off_track_odometry_for_test";

  rclcpp::init(0, nullptr);

  auto test_manager =
    std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  auto test_target_node = std::make_shared<rclcpp::Node>("target_node_for_subscription_test");

  std::optional<Odometry> off_track_odometry;
  test_manager->subscribeOutput<Odometry>(
    off_track_odometry_topic,
    [&off_track_odometry](const Odometry::ConstSharedPtr & msg) { off_track_odometry = *msg; });
  test_manager->testWithOffTrackOdometry(test_target_node, off_track_odometry_topic);
  ASSERT_TRUE(off_track_odometry.has_value());

  rclcpp::shutdown();
}

TEST(PlanningTestManager, CallbackSubscriptionTestWithMsgValidation)
{
  using nav_msgs::msg::Odometry;
  const std::string off_track_odometry_topic = "off_track_odometry_for_test";
  rclcpp::init(0, nullptr);

  auto test_manager =
    std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  auto test_target_node = std::make_shared<rclcpp::Node>("target_node_for_msg_validation_test");

  std::optional<Odometry> off_track_odometry;
  Odometry msg{rosidl_runtime_cpp::MessageInitialization::ZERO};
  msg.pose.pose.position.x = 10.0;
  test_manager->subscribeOutput<Odometry>(
    off_track_odometry_topic,
    [&off_track_odometry](const Odometry::ConstSharedPtr & msg) { off_track_odometry = *msg; });
  test_manager->publishInput(test_target_node, off_track_odometry_topic, msg);
  ASSERT_TRUE(off_track_odometry.has_value());
  EXPECT_DOUBLE_EQ(off_track_odometry->pose.pose.position.x, 10.0);

  rclcpp::shutdown();
}

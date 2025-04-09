// Copyright 2023- Autoware Foundation
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

#include "autoware/localization_util/smart_pose_buffer.hpp"
#include "autoware/localization_util/util_func.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using SmartPoseBuffer = autoware::localization_util::SmartPoseBuffer;

bool compare_pose(
  const PoseWithCovarianceStamped & pose_a, const PoseWithCovarianceStamped & pose_b)
{
  return pose_a.header.stamp == pose_b.header.stamp &&
         pose_a.header.frame_id == pose_b.header.frame_id &&
         pose_a.pose.covariance == pose_b.pose.covariance &&
         pose_a.pose.pose.position.x == pose_b.pose.pose.position.x &&
         pose_a.pose.pose.position.y == pose_b.pose.pose.position.y &&
         pose_a.pose.pose.position.z == pose_b.pose.pose.position.z &&
         pose_a.pose.pose.orientation.x == pose_b.pose.pose.orientation.x &&
         pose_a.pose.pose.orientation.y == pose_b.pose.pose.orientation.y &&
         pose_a.pose.pose.orientation.z == pose_b.pose.pose.orientation.z &&
         pose_a.pose.pose.orientation.w == pose_b.pose.pose.orientation.w;
}

TEST(TestSmartPoseBuffer, interpolate_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double pose_timeout_sec = 10.0;
  const double pose_distance_tolerance_meters = 100.0;
  SmartPoseBuffer smart_pose_buffer(logger, pose_timeout_sec, pose_distance_tolerance_meters);

  // first data
  PoseWithCovarianceStamped::SharedPtr old_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  old_pose_ptr->header.stamp.sec = 10;
  old_pose_ptr->header.stamp.nanosec = 0;
  old_pose_ptr->pose.pose.position.x = 10.0;
  old_pose_ptr->pose.pose.position.y = 20.0;
  old_pose_ptr->pose.pose.position.z = 0.0;
  old_pose_ptr->pose.pose.orientation =
    autoware::localization_util::rpy_deg_to_quaternion(0.0, 0.0, 0.0);
  smart_pose_buffer.push_back(old_pose_ptr);

  // second data
  PoseWithCovarianceStamped::SharedPtr new_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  new_pose_ptr->header.stamp.sec = 20;
  new_pose_ptr->header.stamp.nanosec = 0;
  new_pose_ptr->pose.pose.position.x = 20.0;
  new_pose_ptr->pose.pose.position.y = 40.0;
  new_pose_ptr->pose.pose.position.z = 0.0;
  new_pose_ptr->pose.pose.orientation =
    autoware::localization_util::rpy_deg_to_quaternion(0.0, 0.0, 90.0);
  smart_pose_buffer.push_back(new_pose_ptr);

  // interpolate
  builtin_interfaces::msg::Time target_ros_time_msg;
  target_ros_time_msg.sec = 15;
  target_ros_time_msg.nanosec = 0;
  const std::optional<SmartPoseBuffer::InterpolateResult> & interpolate_result =
    smart_pose_buffer.interpolate(target_ros_time_msg);
  ASSERT_TRUE(interpolate_result.has_value());
  const SmartPoseBuffer::InterpolateResult result = interpolate_result.value();

  // check old
  EXPECT_TRUE(compare_pose(result.old_pose, *old_pose_ptr));

  // check new
  EXPECT_TRUE(compare_pose(result.new_pose, *new_pose_ptr));

  // check interpolated
  EXPECT_EQ(result.interpolated_pose.header.stamp.sec, 15);
  EXPECT_EQ(result.interpolated_pose.header.stamp.nanosec, static_cast<uint32_t>(0));
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.x, 15.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.y, 30.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.z, 0.0);
  const auto rpy = autoware::localization_util::get_rpy(result.interpolated_pose.pose.pose);
  EXPECT_NEAR(rpy.x * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.y * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.z * 180 / M_PI, 45.0, 1e-6);
}

TEST(TestSmartPoseBuffer, empty_buffer)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1.0, 1.0);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 0;
  target_time.nanosec = 0;

  // Test empty buffer
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());

  // Test buffer with only one element
  auto pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  pose_ptr->header.stamp = target_time;
  smart_pose_buffer.push_back(pose_ptr);

  result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());
}

TEST(TestSmartPoseBuffer, timeout_validation)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double timeout = 1.0;  // 1 second timeout
  SmartPoseBuffer smart_pose_buffer(logger, timeout, 100.0);

  // Add two poses with 0.5 sec difference
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 0;
  pose1->header.stamp.nanosec = 0;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 0;
  pose2->header.stamp.nanosec = 500000000;  // 0.5 sec
  smart_pose_buffer.push_back(pose2);

  // Test target time within timeout
  builtin_interfaces::msg::Time target_time1;
  target_time1.sec = 0;
  target_time1.nanosec = 250000000;  // 0.25 sec
  auto result1 = smart_pose_buffer.interpolate(target_time1);
  EXPECT_TRUE(result1.has_value());

  // Test target time beyond timeout
  builtin_interfaces::msg::Time target_time2;
  target_time2.sec = 2;  // 2 sec (beyond 1 sec timeout)
  target_time2.nanosec = 0;
  auto result2 = smart_pose_buffer.interpolate(target_time2);
  EXPECT_FALSE(result2.has_value());
}

TEST(TestSmartPoseBuffer, position_tolerance_validation)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double tolerance = 1.0;  // 1 meter tolerance
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, tolerance);

  // Add two poses within tolerance
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 0;
  pose1->pose.pose.position.x = 0;
  pose1->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 1;
  pose2->pose.pose.position.x = 0.5;  // 0.5m distance
  pose2->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose2);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 0;
  target_time.nanosec = 500000000;  // 0.5 sec
  auto result1 = smart_pose_buffer.interpolate(target_time);
  EXPECT_TRUE(result1.has_value());

  // Add a pose beyond tolerance
  auto pose3 = std::make_shared<PoseWithCovarianceStamped>();
  pose3->header.stamp.sec = 2;
  pose3->pose.pose.position.x = 2.0;  // 2m distance (beyond 1m tolerance)
  pose3->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose3);

  target_time.sec = 1;
  auto result2 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result2.has_value());
}

TEST(TestSmartPoseBuffer, buffer_operations)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Test pop_old
  for (int i = 0; i < 5; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    smart_pose_buffer.push_back(pose);
  }

  builtin_interfaces::msg::Time pop_time;
  pop_time.sec = 2;
  smart_pose_buffer.pop_old(pop_time);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 1;
  auto result1 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result1.has_value());  // Should fail because we popped too much

  target_time.sec = 3;
  auto result2 = smart_pose_buffer.interpolate(target_time);
  EXPECT_TRUE(result2.has_value());

  // Test clear
  smart_pose_buffer.clear();
  auto result3 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result3.has_value());
}

TEST(TestSmartPoseBuffer, non_chronological_timestamps)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Add poses in order
  for (int i = 0; i < 3; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    smart_pose_buffer.push_back(pose);
  }

  // Add pose with older timestamp (should clear buffer)
  auto old_pose = std::make_shared<PoseWithCovarianceStamped>();
  old_pose->header.stamp.sec = 0;
  smart_pose_buffer.push_back(old_pose);

  // Buffer should now only contain the old_pose
  builtin_interfaces::msg::Time target_time;
  target_time.sec = 1;
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());  // Not enough poses in buffer
}

TEST(TestSmartPoseBuffer, target_time_before_first_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Add two poses
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 10;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 20;
  smart_pose_buffer.push_back(pose2);

  // Test target time before first pose
  builtin_interfaces::msg::Time target_time;
  target_time.sec = 5;
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());
}

TEST(TestSmartPoseBuffer, target_time_after_last_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double timeout = 1.0;
  SmartPoseBuffer smart_pose_buffer(logger, timeout, 10.0);

  // Add two poses
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 10;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 11;
  smart_pose_buffer.push_back(pose2);

  // Test target time slightly after last pose (within timeout)
  builtin_interfaces::msg::Time target_time1;
  target_time1.sec = 11;
  target_time1.nanosec = 500000000;  // 11.5 sec
  auto result1 = smart_pose_buffer.interpolate(target_time1);
  EXPECT_TRUE(result1.has_value());

  // Test target time well after last pose (beyond timeout)
  builtin_interfaces::msg::Time target_time2;
  target_time2.sec = 12;  // 12 sec (beyond 1 sec timeout)
  auto result2 = smart_pose_buffer.interpolate(target_time2);
  EXPECT_FALSE(result2.has_value());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

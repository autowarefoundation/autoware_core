// Copyright 2026 TIER IV
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

#include <autoware/planning_topic_converter/path_to_trajectory.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace
{
autoware_planning_msgs::msg::PathPoint make_path_point(
  const double x, const double y, const float longitudinal_velocity,
  const float lateral_velocity, const float heading_rate)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = longitudinal_velocity;
  point.lateral_velocity_mps = lateral_velocity;
  point.heading_rate_rps = heading_rate;
  return point;
}

autoware_planning_msgs::msg::Path make_path(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points)
{
  autoware_planning_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.points = points;
  return path;
}
}  // namespace

// Drives the node over real publish/subscribe so each test body stays a plain Arrange/Act/Assert:
// the fixture owns the ROS context, the executor thread and the test-side pub/sub wiring.
class PathToTrajectoryNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    test_control_node_ = std::make_shared<rclcpp::Node>("test_control_node");
    trajectory_subscription_ =
      test_control_node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        "output/trajectory", 10,
        [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr message) {
          std::lock_guard<std::mutex> lock(message_mutex_);
          received_trajectory_ = message;
        });
    path_publisher_ = test_control_node_->create_publisher<autoware_planning_msgs::msg::Path>(
      "input/path", 10);
    executor_->add_node(test_control_node_);
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  // Bring up the converter node with the required topic parameters and start spinning both nodes.
  void start_converter_node()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"input_topic", "input/path"}, {"output_topic", "output/trajectory"}});
    converter_node_ =
      std::make_shared<autoware::planning_topic_converter::PathToTrajectory>(options);
    executor_->add_node(converter_node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Poll until both endpoints have discovered each other instead of sleeping a fixed duration,
    // because DDS discovery time varies across environments and CI machines.
    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < discovery_deadline) {
      if (
        path_publisher_->get_subscription_count() > 0 &&
        trajectory_subscription_->get_publisher_count() > 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Publish a path and return the converted trajectory, or nullptr if none arrives within the
  // timeout.
  autoware_planning_msgs::msg::Trajectory::SharedPtr publish_and_wait(
    const autoware_planning_msgs::msg::Path & path)
  {
    path_publisher_->publish(path);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(message_mutex_);
        if (received_trajectory_) {
          return received_trajectory_;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::lock_guard<std::mutex> lock(message_mutex_);
    return received_trajectory_;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Node::SharedPtr test_control_node_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    trajectory_subscription_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_publisher_;
  std::shared_ptr<autoware::planning_topic_converter::PathToTrajectory> converter_node_;

  std::mutex message_mutex_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr received_trajectory_;
};

TEST_F(PathToTrajectoryNodeTest, ConvertsPathPointsToTrajectoryPoints)
{
  // Arrange
  start_converter_node();
  const auto path = make_path(
    {make_path_point(0.0, 0.0, 1.0, 0.0, 0.0), make_path_point(1.0, 2.0, 2.5, 0.1, 0.2),
     make_path_point(3.0, 4.0, 3.0, -0.1, -0.05)});

  // Act
  const auto trajectory = publish_and_wait(path);

  // Assert
  ASSERT_NE(trajectory, nullptr) << "Trajectory message was not received within timeout";

  // Header is copied verbatim from the input path.
  EXPECT_EQ(trajectory->header.frame_id, "map");

  ASSERT_EQ(trajectory->points.size(), path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & path_point = path.points[i];
    const auto & traj_point = trajectory->points[i];

    // Pose is mapped directly without any transformation.
    EXPECT_DOUBLE_EQ(traj_point.pose.position.x, path_point.pose.position.x);
    EXPECT_DOUBLE_EQ(traj_point.pose.position.y, path_point.pose.position.y);

    // Longitudinal/lateral velocity and heading rate are mapped directly.
    EXPECT_FLOAT_EQ(traj_point.longitudinal_velocity_mps, path_point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(traj_point.lateral_velocity_mps, path_point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(traj_point.heading_rate_rps, path_point.heading_rate_rps);

    // The conversion does not compute timing, acceleration or steering, so these
    // TrajectoryPoint-only fields stay at their default-constructed value.
    EXPECT_EQ(traj_point.time_from_start.sec, 0);
    EXPECT_EQ(traj_point.time_from_start.nanosec, 0u);
    EXPECT_FLOAT_EQ(traj_point.acceleration_mps2, 0.0f);
    EXPECT_FLOAT_EQ(traj_point.front_wheel_angle_rad, 0.0f);
    EXPECT_FLOAT_EQ(traj_point.rear_wheel_angle_rad, 0.0f);
  }
}

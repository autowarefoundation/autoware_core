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

#include "../src/experimental/scene.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <vector>

namespace
{
geometry_msgs::msg::Point make_geom_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

autoware_internal_planning_msgs::msg::PathPointWithLaneId make_path_point(
  const double x, const double y)
{
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position = make_geom_point(x, y);
  point.lane_ids = {0};
  return point;
}
}  // namespace

namespace autoware::behavior_velocity_planner::experimental
{
class StopLineModuleTest : public ::testing::Test
{
protected:
  // Set up the test case
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_planner_common") +
         "/config/behavior_velocity_planner_common.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});
    node_ = std::make_shared<rclcpp::Node>("test_node", options);

    // Initialize parameters, logger, and clock
    planner_param_.stop_margin = 0.5;
    planner_param_.stop_duration_sec = 2.0;
    planner_param_.hold_stop_margin_distance = 0.5;

    planner_data_ = std::make_shared<autoware::behavior_velocity_planner::PlannerData>(*node_);
    planner_data_->vehicle_info_.max_longitudinal_offset_m = 1.0;

    stop_line_ = lanelet::ConstLineString3d(
      lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 7.0, -1.0, 0.0),
                                lanelet::Point3d(lanelet::utils::getId(), 7.0, 1.0, 0.0)});

    const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> path_points = {
      make_path_point(0.0, 0.0), make_path_point(1.0, 0.0), make_path_point(2.0, 0.0),
      make_path_point(3.0, 0.0), make_path_point(4.0, 0.0), make_path_point(5.0, 0.0),
      make_path_point(6.0, 0.0), make_path_point(7.0, 0.0), make_path_point(8.0, 0.0),
      make_path_point(9.0, 0.0), make_path_point(10.0, 0.0)};
    path_left_bound_ = {make_geom_point(0.0, 1.0), make_geom_point(10.0, 1.0)};
    path_right_bound_ = {make_geom_point(0.0, -1.0), make_geom_point(10.0, -1.0)};

    path_ = *autoware::experimental::trajectory::pretty_build(path_points);

    clock_ = std::make_shared<rclcpp::Clock>();

    module_ = std::make_shared<StopLineModule>(
      1, stop_line_, 0, planner_param_, rclcpp::get_logger("test_logger"), clock_,
      std::make_shared<autoware_utils_debug::TimeKeeper>(),
      std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
        node_.get(), "test_stopline"));
  }

  void TearDown() override { rclcpp::shutdown(); }

  void setState(const StopLineModule::State state) { module_->state_ = state; }
  void setStoppedTime(const std::optional<rclcpp::Time> & time) { module_->stopped_time_ = time; }

  StopLineModule::State getState() const { return module_->state_; }
  std::optional<rclcpp::Time> getStoppedTime() const { return module_->stopped_time_; }

  autoware::behavior_velocity_planner::Trajectory path_;
  std::vector<geometry_msgs::msg::Point> path_left_bound_;
  std::vector<geometry_msgs::msg::Point> path_right_bound_;
  StopLineModule::PlannerParam planner_param_{};
  lanelet::ConstLineString3d stop_line_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<StopLineModule> module_;
  std::shared_ptr<autoware::behavior_velocity_planner::PlannerData> planner_data_;

  rclcpp::Node::SharedPtr node_;
};

TEST_F(StopLineModuleTest, TestGetEgoAndStopPoint)
{
  // Prepare parameters

  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 5.0;
  ego_pose.position.y = 1.0;

  {  // Test for APPROACH state
    setState(StopLineModule::State::APPROACH);
    // Execute the function
    const auto [ego_s, stop_point_s] = module_->getEgoAndStopPoint(
      path_, path_left_bound_, path_right_bound_, ego_pose, *planner_data_);

    // Verify results
    EXPECT_DOUBLE_EQ(ego_s, 5.0);
    EXPECT_DOUBLE_EQ(stop_point_s.value(), 7.0 - 0.5 - 1.0);
  }

  {  // Test for STOPPED state
    setState(StopLineModule::State::STOPPED);
    // Execute the function
    const auto [ego_s, stop_point_s] = module_->getEgoAndStopPoint(
      path_, path_left_bound_, path_right_bound_, ego_pose, *planner_data_);

    EXPECT_TRUE(stop_point_s.has_value());
    EXPECT_DOUBLE_EQ(ego_s, 5.0);
    EXPECT_DOUBLE_EQ(stop_point_s.value(), 5.0);
  }
}

TEST_F(StopLineModuleTest, TestUpdateStateAndStoppedTime)
{
  constexpr auto distance_to_stop_point = 0.1;
  constexpr auto is_vehicle_stopped = true;

  setState(StopLineModule::State::APPROACH);

  // Simulate clock progression
  auto test_start_time = clock_->now();
  setStoppedTime(test_start_time);

  // Execute the function
  module_->updateStateAndStoppedTime(test_start_time, distance_to_stop_point, is_vehicle_stopped);

  // Verify state transition to STOPPED
  EXPECT_EQ(getState(), StopLineModule::State::STOPPED);
  EXPECT_TRUE(getStoppedTime().has_value());

  // Simulate time elapsed to exceed stop duration
  auto now = test_start_time + rclcpp::Duration::from_seconds(3.0);
  module_->updateStateAndStoppedTime(now, distance_to_stop_point, is_vehicle_stopped);

  // Verify state transition to START
  EXPECT_EQ(getState(), StopLineModule::State::START);
  EXPECT_FALSE(getStoppedTime().has_value());
}
}  // namespace autoware::behavior_velocity_planner::experimental

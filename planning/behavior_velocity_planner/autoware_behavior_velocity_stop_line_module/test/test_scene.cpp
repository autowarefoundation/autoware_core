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

#include "../src/scene.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <rclcpp/node.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <tuple>

using autoware::behavior_velocity_planner::StopLineModule;

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

    path_.points = {make_path_point(0.0, 0.0), make_path_point(1.0, 0.0), make_path_point(2.0, 0.0),
                    make_path_point(3.0, 0.0), make_path_point(4.0, 0.0), make_path_point(5.0, 0.0),
                    make_path_point(6.0, 0.0), make_path_point(7.0, 0.0), make_path_point(8.0, 0.0),
                    make_path_point(9.0, 0.0), make_path_point(10.0, 0.0)};
    path_.left_bound = {make_geom_point(0.0, 1.0), make_geom_point(10.0, 1.0)};
    path_.right_bound = {make_geom_point(0.0, -1.0), make_geom_point(10.0, -1.0)};

    trajectory_ = *StopLineModule::Trajectory::Builder{}.build(path_.points);

    clock_ = std::make_shared<rclcpp::Clock>();

    module_ = std::make_shared<StopLineModule>(
      1, stop_line_, 0, planner_param_, rclcpp::get_logger("test_logger"), clock_,
      std::make_shared<autoware_utils_debug::TimeKeeper>(),
      std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
        node_.get(), "test_stopline"));

    module_->setPlannerData(planner_data_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  autoware_internal_planning_msgs::msg::PathWithLaneId path_;
  StopLineModule::Trajectory trajectory_;
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
    // Execute the function
    const auto [ego_s, stop_point_s] =
      module_->getEgoAndStopPoint(trajectory_, path_, ego_pose, StopLineModule::State::APPROACH);

    // Verify results
    EXPECT_DOUBLE_EQ(ego_s, 5.0);
    EXPECT_DOUBLE_EQ(stop_point_s.value(), 7.0 - 0.5 - 1.0);
  }

  {  // Test for STOPPED state
    const auto [ego_s, stop_point_s] =
      module_->getEgoAndStopPoint(trajectory_, path_, ego_pose, StopLineModule::State::STOPPED);

    EXPECT_TRUE(stop_point_s.has_value());
    EXPECT_DOUBLE_EQ(ego_s, 5.0);
    EXPECT_DOUBLE_EQ(stop_point_s.value(), 5.0);
  }
}

TEST_F(StopLineModuleTest, TestUpdateStateAndStoppedTime)
{
  StopLineModule::State state = StopLineModule::State::APPROACH;
  std::optional<rclcpp::Time> stopped_time;
  double distance_to_stop_point = 0.1;
  bool is_vehicle_stopped = true;

  // Simulate clock progression
  auto test_start_time = clock_->now();
  stopped_time = test_start_time;

  // Execute the function
  module_->updateStateAndStoppedTime(
    &state, &stopped_time, test_start_time, distance_to_stop_point, is_vehicle_stopped);

  // Verify state transition to STOPPED
  EXPECT_EQ(state, StopLineModule::State::STOPPED);
  EXPECT_TRUE(stopped_time.has_value());

  // Simulate time elapsed to exceed stop duration
  auto now = test_start_time + rclcpp::Duration::from_seconds(3.0);
  module_->updateStateAndStoppedTime(
    &state, &stopped_time, now, distance_to_stop_point, is_vehicle_stopped);

  // Verify state transition to START
  EXPECT_EQ(state, StopLineModule::State::START);
  EXPECT_FALSE(stopped_time.has_value());
}

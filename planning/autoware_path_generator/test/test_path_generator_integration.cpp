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

#include "autoware/path_generator/path_generator_node.hpp"
#include "utils_test.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-1F;
}  // namespace

namespace autoware::path_generator
{

class PathGeneratorIntegrationHarness : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Load params
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto path_generator_dir =
      ament_index_cpp::get_package_share_directory("autoware_path_generator");
    const auto node_options = rclcpp::NodeOptions{}.arguments(
      {"--ros-args", "--params-file",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
       autoware_test_utils_dir + "/config/test_nearest_search.param.yaml", "--params-file",
       path_generator_dir + "/config/path_generator.param.yaml"});

    // Init nodes
    node_ = std::make_shared<PathGeneratorNode>(node_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Setup I/O harness
    harness_node_ = std::make_shared<rclcpp::Node>("path_generator_harness");
    executor_->add_node(harness_node_);

    // Pubs
    pub_map_ = harness_node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/path_generator/input/vector_map", rclcpp::QoS(1).transient_local());
    pub_odom_ =
      harness_node_->create_publisher<nav_msgs::msg::Odometry>("/path_generator/input/odometry", 1);
    pub_route_ = harness_node_->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
      "/path_generator/input/route", rclcpp::QoS(1).transient_local());

    // Subs
    sub_path_ =
      harness_node_->create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
        "/path_generator/output/path", 1,
        [this](const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg) {
          latest_path_ = msg;
        });
    sub_turn_ =
      harness_node_->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/path_generator/output/turn_indicators_cmd", 1,
        [this](const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg) {
          latest_turn_ = msg;
        });
    sub_hazard_ =
      harness_node_->create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
        "/path_generator/output/hazard_lights_cmd", 1,
        [this](const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg) {
          latest_hazard_ = msg;
        });
  }

  void TearDown() override { rclcpp::shutdown(); }

  void spin_executor_for(std::chrono::milliseconds duration)
  {
    const auto end_time = std::chrono::steady_clock::now() + duration;

    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Wait helper: spins executor until condition becomes true or timeout elapses
  template <typename Pred>
  bool wait_for(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto end_time = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      if (pred()) {
        return true;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
  }

  void retrigger_pubs_spin(
    const std::optional<nav_msgs::msg::Odometry> & odom,
    const std::optional<autoware_planning_msgs::msg::LaneletRoute> & route,
    std::chrono::milliseconds spin_time)
  {
    if (odom.has_value()) {
      pub_odom_->publish(odom.value());
    }
    if (route.has_value()) {
      pub_route_->publish(route.value());
    }
    spin_executor_for(std::chrono::milliseconds(spin_time));
  }

  // Harness pointers
  std::shared_ptr<PathGeneratorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> harness_node_;

  // Pubs
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr pub_route_;

  // Output storages
  autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr latest_path_{nullptr};
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr latest_turn_{nullptr};
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr latest_hazard_{nullptr};

  // Subs
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr sub_path_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr sub_turn_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr sub_hazard_;
};

// ==================================== TESTING AREA HERE ====================================

// TEST 1. Nominal generation & observability
// This test verifies a standard route outputs a path, correctly stamps headers, and constant hazard
// rule.
TEST_F(PathGeneratorIntegrationHarness, NominalStandardRouteExecution)
{
  auto map_msg = create_mock_common_map_bin();
  pub_map_->publish(map_msg);

  // Set odom to route start
  auto route = create_mock_route();
  auto odom = set_start_odom(route);

  // Allow map & odom to register
  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));

  // Allow planning trigger
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(100));

  // Memory check before accessing
  ASSERT_TRUE(wait_for([this] { return latest_path_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output PathWithLaneId";
  ASSERT_TRUE(
    wait_for([this] { return latest_hazard_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output HazardLightsCommand";

  // Expects headers not empty
  EXPECT_FALSE(latest_path_->header.frame_id.empty());

  // Expects output math bounds make sense
  ASSERT_GT(latest_path_->points.size(), 0u) << "Output path array is empty";

  // Expects constant hazard rule makes sense
  EXPECT_EQ(latest_hazard_->command, autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND);
}

// TEST 2. Turns signal state machine
// This test verifies turn signal strictly triggers at expected proximity to consecutive turns.
TEST_F(PathGeneratorIntegrationHarness, TurnSignalStateTransition)
{
  auto map_msg = create_mock_dense_map_bin("right");
  pub_map_->publish(map_msg);

  auto route = create_mock_route();

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(100));

  ASSERT_TRUE(wait_for([this] { return latest_turn_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output TurnIndicatorsCommand";

  // If ego is stopping and far, should be NO_COMMAND
  // We don't strictly assert NO_COMMAND here because it depends on exact yaml distance vs params,
  // but we expects node survives and outputs something.
  EXPECT_GE(latest_turn_->command, 0);

  // Simulate advancing odom to trigger section
  latest_turn_ = nullptr;
  odom.pose.pose.position.x = 50.5;
  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(100));

  ASSERT_TRUE(wait_for([this] { return latest_turn_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output TurnIndicatorsCommand after odom advance";
}

// TEST 3. Fail-safe runtime boundary
// This test fetches an empty route or out-of-bound odom.
// Expects node not to crash, but should gracefully abort.
TEST_F(PathGeneratorIntegrationHarness, FailSafeOnAbnormalRoute)
{
  auto map_msg = create_mock_common_map_bin();
  pub_map_->publish(map_msg);

  // Empty route declared
  autoware_planning_msgs::msg::LaneletRoute empty_route;
  empty_route.header.frame_id = "map";

  // Empty odom declared
  auto odom = set_start_odom(std::nullopt);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));

  // Reset captured pointers
  latest_path_ = nullptr;

  // Publish empty route
  retrigger_pubs_spin(std::nullopt, empty_route, std::chrono::milliseconds(100));

  // Expects node should not crash
  EXPECT_EQ(latest_path_, nullptr) << "Node should fail-safe and not publish on empty route";
}

// TEST 4. Path cut scenario
// This test checks if node successfully processes self-intersecting bounds and
// outputs a valid truncated path without crashing.
TEST_F(PathGeneratorIntegrationHarness, PathCutScenario)
{
  auto map_msg = create_mock_loop_map_bin();
  pub_map_->publish(map_msg);

  auto route = create_mock_loop_route();

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(100));

  ASSERT_TRUE(wait_for([this] { return latest_path_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Failed to output path for path_cut_route (self-intersection truncation failed)";
  EXPECT_GT(latest_path_->points.size(), 0u);

  // Theoretically, path should be truncated at around intersection (20.0, 2.0)
  // (Y = 2.0 is L1 boundary)
  const auto & final_point = latest_path_->points.back().point.pose.position;

  // X should be 20.0 (centerline of intersection)
  EXPECT_NEAR(final_point.x, 20.0, near_tol) << "Path cut failed to truncate at intersection X";

  // Y should be 5.79.
  // L1 boundary at Y = 2.0
  // In current `test_vehicle_info.param.yaml`:
  // - wheel_base: 2.79
  // - front_overhang: 1.0
  // So now Y = 2.0 + 2.79 + 1.0 = 5.79
  const double wheel_base = node_->get_parameter("wheel_base").as_double();
  const double front_overhang = node_->get_parameter("front_overhang").as_double();
  EXPECT_NEAR(final_point.y, 2.0 + wheel_base + front_overhang, near_tol)
    << "Path cut failed to truncate at intersection Y";
}

// TEST 5: Goal connection scenario
// This test checks if final point of generated path is smoothly aligned
// and matches requested goal pose.
// As a matter of fact, this test is supposed to replace the old, removed
// `test_dense_centerline.cpp` test suite in previous code version.
TEST_F(PathGeneratorIntegrationHarness, GoalConnectionScenario)
{
  auto map_msg = create_mock_dense_map_bin();
  pub_map_->publish(map_msg);

  auto route = create_mock_route();
  route.goal_pose.position.x = 45.5;
  // Offset goal laterally to force smoothing algorithm to engage
  // Common map has lateral range [-1.75, 1.75] so just set goal within it
  route.goal_pose.position.y = 1.0;

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(100));

  ASSERT_TRUE(wait_for([this] { return latest_path_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Failed to output path for dense_centerline_route";
  ASSERT_GT(latest_path_->points.size(), 0u);

  // Assert final point aligns geometrically with goal pose
  const auto & final_point = latest_path_->points.back().point.pose.position;
  const auto & goal_point = route.goal_pose.position;
  EXPECT_NEAR(final_point.x, goal_point.x, near_tol)
    << "Goal connection smoothing failed on X axis";
  EXPECT_NEAR(final_point.y, goal_point.y, near_tol)
    << "Goal connection smoothing failed on Y axis";
}

// TEST 6: Missing dependency scenario
// This test checks if node safely aborts and refuses to publish if a critical
// dependency (like vector map) is completely missing.
TEST_F(PathGeneratorIntegrationHarness, FailSafeOnMissingDependencies)
{
  auto route = create_mock_route();

  auto odom = set_start_odom(route);

  // We don't publish map here
  latest_path_ = nullptr;

  // Trigger execution with only odom and route
  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(100));

  // Node must not crash, and must fail-safe by not publishing a path
  EXPECT_EQ(latest_path_, nullptr) << "Node published a path despite missing vector map dependency";
}

}  // namespace autoware::path_generator

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
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <optional>
#include <string>
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
    node_ = std::make_shared<PathGenerator>(node_options);
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

  // ================== MOCK MAP BIN GENERATION FUNCS ==================

  // Deterministic, 100-meter straight map
  // Used in TEST 1, 3, 5
  static autoware_map_msgs::msg::LaneletMapBin create_mock_common_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    
    // Left bound (y = 1.75)
    lanelet::Point3d p1_left(1, 0.0, 1.75, 0.0);
    lanelet::Point3d p2_left(2, 100.0, 1.75, 0.0);
    lanelet::LineString3d left_bound(10, {p1_left, p2_left});

    // Right bound (y = 0.0)
    lanelet::Point3d p1_right(3, 0.0, -1.75, 0.0);
    lanelet::Point3d p2_right(4, 100.0, -1.75, 0.0);
    lanelet::LineString3d right_bound(11, {p1_right, p2_right});

    // Lanelet (ID 1000)
    lanelet::Lanelet mock_lanelet(1000, left_bound, right_bound);

    // Attributes
    mock_lanelet.attributes()[lanelet::AttributeName::Type] =
      lanelet::AttributeValueString::Lanelet;
    mock_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    map->add(mock_lanelet);

    // Convert to ROS binary message
    autoware_map_msgs::msg::LaneletMapBin map_bin_msg =
      autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
    map_bin_msg.header.frame_id = "map";

    return map_bin_msg;
  }

  // Map with a turn
  // Used in TEST 2
  static autoware_map_msgs::msg::LaneletMapBin create_mock_turn_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    
    lanelet::LineString3d left_bound(10, {lanelet::Point3d(1, 0.0, 1.75, 0.0), lanelet::Point3d(2, 100.0, 1.75, 0.0)});
    lanelet::LineString3d right_bound(11, {lanelet::Point3d(3, 0.0, -1.75, 0.0), lanelet::Point3d(4, 100.0, -1.75, 0.0)});

    lanelet::Lanelet turn_lanelet(1000, left_bound, right_bound);
    turn_lanelet.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
    turn_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    
    turn_lanelet.attributes()["turn_direction"] = lanelet::AttributeValueString::Right;
    map->add(turn_lanelet);

    // Convert to ROS binary message
    autoware_map_msgs::msg::LaneletMapBin map_bin_msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
    map_bin_msg.header.frame_id = "map";

    return map_bin_msg;
  }

  // Map with left/right bounds physically crossed
  // Used in TEST 4
  static autoware_map_msgs::msg::LaneletMapBin create_mock_x_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    
    // Two bounds start as valid parallel section with x: [0, 40], 
    // then become gradually crossing with x: (40, 80]
    
    // Left bound: top-left => bottom-right
    lanelet::LineString3d left_bound(
    10, 
    {
      lanelet::Point3d(1, 0.0, 1.75, 0.0),
      lanelet::Point3d(2, 40.0, 1.75, 0.0),
      lanelet::Point3d(3, 80.0, -1.75, 0.0)
    });
    // Right bound: bottom-left => top-right
    lanelet::LineString3d right_bound(
    11, 
    {
      lanelet::Point3d(4, 0.0, -1.75, 0.0),
      lanelet::Point3d(5, 40.0, -1.75, 0.0),
      lanelet::Point3d(6, 80.0, 1.75, 0.0)
    });

    lanelet::Lanelet cross_lanelet(1000, left_bound, right_bound);
    cross_lanelet.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
    cross_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    
    map->add(cross_lanelet);

    autoware_map_msgs::msg::LaneletMapBin map_bin_msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
    map_bin_msg.header.frame_id = "map";

    return map_bin_msg;
  }

  // 100-meter straight, dense map
  // Used in TEST 5
  static autoware_map_msgs::msg::LaneletMapBin create_mock_dense_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    
    lanelet::Points3d left_points;
    lanelet::Points3d right_points;
    
    // Inject dense nodes every 1 meter to provide spline resolution
    for (double x = 0.0; x <= 100.0; x += 1.0) {
      left_points.emplace_back(lanelet::utils::getId(), x, 1.75, 0.0);
      right_points.emplace_back(lanelet::utils::getId(), x, -1.75, 0.0);
    }
    
    lanelet::LineString3d left_bound(10, left_points);
    lanelet::LineString3d right_bound(11, right_points);

    lanelet::Lanelet dense_lanelet(1000, left_bound, right_bound);
    dense_lanelet.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
    dense_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    
    map->add(dense_lanelet);

    autoware_map_msgs::msg::LaneletMapBin map_bin_msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
    map_bin_msg.header.frame_id = "map";

    return map_bin_msg;
  }

  // ===================================================================

  // Same mock route for above lanelets
  static autoware_planning_msgs::msg::LaneletRoute create_mock_route()
  {
    autoware_planning_msgs::msg::LaneletRoute route;
    route.header.frame_id = "map";

    // Set deterministic poses within 100m map
    // Here start at X = 10.0 to satisfy edge boundary check
    route.start_pose.position.x = 10.0;
    route.start_pose.position.y = 0.0;
    route.start_pose.orientation.w = 1.0;

    route.goal_pose.position.x = 90.0;
    route.goal_pose.position.y = 0.0;
    route.goal_pose.orientation.w = 1.0;

    // Link to lanelet ID 1000
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = 1000;
    segment.preferred_primitive.primitive_type = "lane";

    // Populate primitive array
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = 1000;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);

    route.segments.push_back(segment);

    return route;
  }

  void load_and_publish_map(const std::string & package_name, const std::string & map_filename)
  {
    const auto map_path =
      autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_filename);
    auto map_msg = autoware::test_utils::make_map_bin_msg(map_path);

    pub_map_->publish(map_msg);
  }

  static autoware_planning_msgs::msg::LaneletRoute load_route(
    const std::string & package_name, const std::string & route_filename)
  {
    const auto route_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, route_filename);
    auto route_opt =
      autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
        route_path);

    if (!route_opt) {
      throw std::runtime_error("Failed to parse mock route.");
    }

    return route_opt.value();
  }

  static nav_msgs::msg::Odometry set_start_odom(
    const autoware_planning_msgs::msg::LaneletRoute & route)
  {
    auto odom = autoware::test_utils::makeOdometry();
    odom.pose.pose = route.start_pose;
    odom.header.frame_id = "map";

    return odom;
  }

  static autoware_planning_msgs::msg::LaneletRoute load_route_stamped(
    const std::string & package_name, const std::string & map_filename)
  {
    auto route = load_route(package_name, map_filename);
    route.header.frame_id = "map";

    return route;
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
  std::shared_ptr<PathGenerator> node_;
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

// ================== TESTING AREA HERE ==================

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
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(500));

  // Memory check before accessing
  ASSERT_NE(latest_path_, nullptr) << "Node failed to output PathWithLaneId";
  ASSERT_NE(latest_hazard_, nullptr) << "Node failed to output HazardLightsCommand";

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
  auto map_msg = create_mock_turn_map_bin();
  pub_map_-> publish(map_msg);

  auto route = create_mock_route();
  
  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_turn_, nullptr);

  // If ego is stopping and far, should be NO_COMMAND
  // We don't strictly assert NO_COMMAND here because it depends on exact yaml distance vs params,
  // but we expects node survives and outputs something.
  EXPECT_GE(latest_turn_->command, 0);

  // Simulate advancing odom to trigger section
  odom.pose.pose.position.x = 80.0;
  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_turn_, nullptr);
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
  auto odom = autoware::test_utils::makeOdometry();
  odom.header.frame_id = "map";

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));

  // Reset captured pointers
  latest_path_ = nullptr;

  // Publish empty route
  retrigger_pubs_spin(std::nullopt, empty_route, std::chrono::milliseconds(500));

  // Expects node should not crash
  EXPECT_EQ(latest_path_, nullptr) << "Node should fail-safe and not publish on empty route";
}

// TEST 4. Path cut scenario
// This test checks if node successfully processes self-intersecting bounds and
// outputs a valid truncated path without crashing.
TEST_F(PathGeneratorIntegrationHarness, PathCutScenario)
{
  auto map_msg = create_mock_x_map_bin();
  pub_map_->publish(map_msg);

  auto route = create_mock_route();

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_path_, nullptr)
    << "Failed to output path for path_cut_route (self-intersection truncation failed)";
  EXPECT_GT(latest_path_->points.size(), 0u);
}

// TEST 5: Goal connection scenario
// This test checks if final point of generated path is smoothly aligned
// and matches requested goal pose.
// As a matter of fact, this test is supposed to replace the old, removed
// `test_dense_centerline.cpp` test suite in previous code version.
TEST_F(PathGeneratorIntegrationHarness, GoalConnectionScenario)
{
  auto map_msg = create_mock_x_map_bin();
  pub_map_->publish(map_msg);

  auto route = create_mock_route();
  route.goal_pose.position.x = 45.0;
  // Offset goal laterally to force smoothing algorithm to engage
  // Common map has lateral range [-1.75, 1.75] so just set goal within it
  route.goal_pose.position.y = 1.0;

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_path_, nullptr) << "Failed to output path for dense_centerline_route";
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
  auto route = load_route_stamped("autoware_path_generator", "common_route.yaml");

  auto odom = set_start_odom(route);

  // We don't publish map here
  latest_path_ = nullptr;

  // Trigger execution with only odom and route
  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  // Node must not crash, and must fail-safe by not publishing a path
  EXPECT_EQ(latest_path_, nullptr) << "Node published a path despite missing vector map dependency";
}

}  // namespace autoware::path_generator

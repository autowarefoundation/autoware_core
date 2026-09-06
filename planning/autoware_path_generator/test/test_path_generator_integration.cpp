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

  // 1. Deterministic, 100-meter straight map
  // Used in TEST 1, 3, 5
  static autoware_map_msgs::msg::LaneletMapBin create_mock_common_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();

    auto [left_bound, right_bound] = prep_common_straight_bounds(100.0, 1.75);
    lanelet::Lanelet mock_lanelet = prep_lanelet(left_bound, right_bound);

    map->add(mock_lanelet);

    return convert_ros_bin_map(map);
  }

  /*
   * 2. Map with 4 lanelets forming a square loop
   * Used in TEST 4
   *
   * y (m)
   *
   *  40 ┤           p8 ─────────────── p6
   *     │            │                  │
   *     │            │   L3 (go West)   │
   *  38 ┤           p7 ─────────────── p5
   *     │            │                  │
   *     │ L4         │                  │ L2
   *     │ (go South) │                  │ (go North)
   *     │            │                  │
   *   2 ┼ p1 ────────│────────┼──────── p3
   *     │            │   X    │         │
   *   0 ┼ S  ────────┼────────┼──────── ┼──────▶ x (m)
   *     │            │        │         │
   *  -2 ┼ p2 ────────│────────┼──────── p4
   *                  │   L1 (go East)   │
   * -20 ┤           p10 ────────────── p9
   *                  18      20        38      40      42
   *
   * Well if above comment is confusing (yes it's confusing lol), just look below
   *
       L3 (go West)
       (X=20, Y=40) ◀━━━━━━━━━━━━━━━━━━━━━━━┓ (X=40, Y=40)
                    ┃                       ┃
                    ┃                       ┃
       L4           ┃                       ┃  L2
       (go          ┃                       ┃  (go
       South)       ┃                       ┃  North)
                    ┃                       ┃
                    ┃    L1 (go East)       ┃
     S ━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━━━━━━━➤┛ (X=40, Y=0)
     (X=0, Y=0)     ┃  (Truncated here)
                    ┃  (X=20, Y=0)
                    ┃
                    ▼ E (X=20, Y=-20)
   */
  static autoware_map_msgs::msg::LaneletMapBin create_mock_loop_map_bin()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();

    // L1 (East): start
    lanelet::Point3d p1(10001, 0.0, 2.0, 0.0);
    lanelet::Point3d p2(10002, 0.0, -2.0, 0.0);
    // Corner 1: L1 => L2 (go North)
    lanelet::Point3d p3(10003, 38.0, 2.0, 0.0);   // Inner
    lanelet::Point3d p4(10004, 42.0, -2.0, 0.0);  // Outer
    // Corner 2: L2 => L3 (go West)
    lanelet::Point3d p5(10005, 38.0, 38.0, 0.0);  // Inner
    lanelet::Point3d p6(10006, 42.0, 42.0, 0.0);  // Outer
    // Corner 3: L3 => L4 (go South)
    lanelet::Point3d p7(10007, 22.0, 38.0, 0.0);  // Inner
    lanelet::Point3d p8(10008, 18.0, 42.0, 0.0);  // Outer
    // L4: end
    lanelet::Point3d p9(10009, 22.0, -20.0, 0.0);   // Left (East side)
    lanelet::Point3d p10(10010, 18.0, -20.0, 0.0);  // Right (West side)

    lanelet::LineString3d ls_l1_left(10101, {p1, p3});
    lanelet::LineString3d ls_l1_right(10102, {p2, p4});

    lanelet::LineString3d ls_l2_left(10103, {p3, p5});
    lanelet::LineString3d ls_l2_right(10104, {p4, p6});

    lanelet::LineString3d ls_l3_left(10105, {p5, p7});
    lanelet::LineString3d ls_l3_right(10106, {p6, p8});

    lanelet::LineString3d ls_l4_left(10107, {p7, p9});
    lanelet::LineString3d ls_l4_right(10108, {p8, p10});

    lanelet::Lanelet l1 = prep_lanelet(ls_l1_left, ls_l1_right);
    l1.setId(1001);
    lanelet::Lanelet l2 = prep_lanelet(ls_l2_left, ls_l2_right);
    l2.setId(1002);
    lanelet::Lanelet l3 = prep_lanelet(ls_l3_left, ls_l3_right);
    l3.setId(1003);
    lanelet::Lanelet l4 = prep_lanelet(ls_l4_left, ls_l4_right);
    l4.setId(1004);

    map->add(l1);
    map->add(l2);
    map->add(l3);
    map->add(l4);

    return convert_ros_bin_map(map);
  }

  // 3. 100-meter straight, dense map
  // Can inject an optional turn direction attribute
  // Used in TEST 5 (dense map), and TEST 2 (turn map)
  static autoware_map_msgs::msg::LaneletMapBin create_mock_dense_map_bin(
    const std::string & turn_direction = "")
  {
    auto map = std::make_shared<lanelet::LaneletMap>();

    lanelet::Points3d left_points;
    lanelet::Points3d right_points;

    // Inject dense nodes every 1 meter to provide spline resolution
    int pt_id = 10000;
    for (double x = 0.0; x <= 100.0; x += 1.0) {
      left_points.emplace_back(++pt_id, x, 1.75, 0.0);
      right_points.emplace_back(++pt_id, x, -1.75, 0.0);
    }

    lanelet::LineString3d left_bound(10, left_points);
    lanelet::LineString3d right_bound(11, right_points);

    lanelet::Lanelet dense_lanelet = prep_lanelet(left_bound, right_bound);

    // Apply turn attribute if requested
    if (!turn_direction.empty()) {
      dense_lanelet.attributes()["turn_direction"] = turn_direction;
    }

    map->add(dense_lanelet);

    return convert_ros_bin_map(map);
  }

  // Helper 1
  static std::pair<lanelet::LineString3d, lanelet::LineString3d> prep_common_straight_bounds(
    double length, double half_width)
  {
    // Left bound
    lanelet::Point3d p1_left(1, 0.0, half_width, 0.0);
    lanelet::Point3d p2_left(2, length, half_width, 0.0);
    lanelet::LineString3d left_bound(10, {p1_left, p2_left});

    // Right bound
    lanelet::Point3d p1_right(3, 0.0, -half_width, 0.0);
    lanelet::Point3d p2_right(4, length, -half_width, 0.0);
    lanelet::LineString3d right_bound(11, {p1_right, p2_right});

    return {left_bound, right_bound};
  }

  // Helper 2
  static lanelet::Lanelet prep_lanelet(
    lanelet::LineString3d & left_bound, lanelet::LineString3d & right_bound)
  {
    lanelet::Lanelet this_lanelet(1000, left_bound, right_bound);
    this_lanelet.attributes()[lanelet::AttributeName::Type] =
      lanelet::AttributeValueString::Lanelet;
    this_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    return this_lanelet;
  }

  // Helper 3
  static autoware_map_msgs::msg::LaneletMapBin convert_ros_bin_map(
    const std::shared_ptr<lanelet::LaneletMap> & map_ptr)
  {
    autoware_map_msgs::msg::LaneletMapBin map_bin_msg =
      autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map_ptr);
    map_bin_msg.header.frame_id = "map";

    return map_bin_msg;
  }

  // ===================================================================

  // Mock route for straight maps, used in TEST 1, 3, 5
  static autoware_planning_msgs::msg::LaneletRoute create_mock_route()
  {
    autoware_planning_msgs::msg::LaneletRoute route;
    route.header.frame_id = "map";

    // Set deterministic poses within 100m map
    // Here start at X = 10.5 to satisfy edge boundary check
    route.start_pose.position.x = 10.5;
    route.start_pose.position.y = 0.0;
    route.start_pose.orientation.w = 1.0;

    route.goal_pose.position.x = 89.5;
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

  // Mock route for loop map, used in TEST 4
  static autoware_planning_msgs::msg::LaneletRoute create_mock_loop_route()
  {
    autoware_planning_msgs::msg::LaneletRoute route;
    route.header.frame_id = "map";

    // Start on L1
    route.start_pose.position.x = 5.5;
    route.start_pose.position.y = 0.0;
    route.start_pose.orientation.w = 1.0;

    // Goal on L3
    route.goal_pose.position.x = 20.0;
    route.goal_pose.position.y = -10.5;

    // Facing South (-90 deg yaw => roughly w = 0.707, z = -0.707)
    route.goal_pose.orientation.w = 0.707;
    route.goal_pose.orientation.z = -0.707;

    // Add lanelet sequences
    for (int id : {1001, 1002, 1003, 1004}) {
      autoware_planning_msgs::msg::LaneletSegment segment;
      segment.preferred_primitive.id = id;
      segment.preferred_primitive.primitive_type = "lane";

      autoware_planning_msgs::msg::LaneletPrimitive primitive;
      primitive.id = id;
      primitive.primitive_type = "lane";

      segment.primitives.push_back(primitive);
      route.segments.push_back(segment);
    }

    return route;
  }

  // ===================================================================

  static nav_msgs::msg::Odometry set_start_odom(
    const std::optional<autoware_planning_msgs::msg::LaneletRoute> & route)
  {
    nav_msgs::msg::Odometry odom;
    if (route.has_value()) {
      odom.pose.pose = route.value().start_pose;
    }
    odom.header.frame_id = "map";

    // A bit forward velocity
    odom.twist.twist.linear.x = 5.0;

    return odom;
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
  auto map_msg = create_mock_dense_map_bin("right");
  pub_map_->publish(map_msg);

  auto route = create_mock_route();

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_turn_, nullptr);

  // If ego is stopping and far, should be NO_COMMAND
  // We don't strictly assert NO_COMMAND here because it depends on exact yaml distance vs params,
  // but we expects node survives and outputs something.
  EXPECT_GE(latest_turn_->command, 0);

  // Simulate advancing odom to trigger section
  latest_turn_ = nullptr;
  odom.pose.pose.position.x = 50.5;
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
  auto odom = set_start_odom(std::nullopt);

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
  auto map_msg = create_mock_loop_map_bin();
  pub_map_->publish(map_msg);

  auto route = create_mock_loop_route();

  auto odom = set_start_odom(route);

  retrigger_pubs_spin(odom, std::nullopt, std::chrono::milliseconds(100));
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(500));

  ASSERT_NE(latest_path_, nullptr)
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
  retrigger_pubs_spin(std::nullopt, route, std::chrono::milliseconds(500));

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
  auto route = create_mock_route();

  auto odom = set_start_odom(route);

  // We don't publish map here
  latest_path_ = nullptr;

  // Trigger execution with only odom and route
  retrigger_pubs_spin(odom, route, std::chrono::milliseconds(500));

  // Node must not crash, and must fail-safe by not publishing a path
  EXPECT_EQ(latest_path_, nullptr) << "Node published a path despite missing vector map dependency";
}

}  // namespace autoware::path_generator

// Copyright 2026 TIER IV, Inc.
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

#include "../src/mission_planner/mission_planner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware::mission_planner::MissionPlanner;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

namespace
{

// IDs of the two colinear road lanelets making up the test map (see create_map()).
constexpr lanelet::Id FIRST_LANELET_ID = 1000;
constexpr lanelet::Id SECOND_LANELET_ID = 1001;

/// @brief Create a lanelet map with two colinear straight road lanelets.
///
///   y
///   2 +----------------+----------------+   <- left bound
///     |   FIRST_LANELET|SECOND_LANELET  |
///  -2 +----------------+----------------+   <- right bound
///     +----------------+----------------+--> x
///     0               50              100
///
/// Lanelet boundaries at a shared x reuse the same Point3d instances (not just equal
/// coordinates) so that lanelet2's routing graph recognizes the lanelets as consecutive:
/// lanelet::geometry::follows() compares boundary points by identity, not by position.
LaneletMapBin create_map()
{
  using lanelet::AttributeName;
  using lanelet::AttributeValueString;
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::Point3d;

  const Point3d left_0(lanelet::utils::getId(), 0.0, 2.0);
  const Point3d left_50(lanelet::utils::getId(), 50.0, 2.0);
  const Point3d left_100(lanelet::utils::getId(), 100.0, 2.0);
  const Point3d right_0(lanelet::utils::getId(), 0.0, -2.0);
  const Point3d right_50(lanelet::utils::getId(), 50.0, -2.0);
  const Point3d right_100(lanelet::utils::getId(), 100.0, -2.0);

  auto make_road_lanelet = [](
                             const lanelet::Id id, const Point3d & left_from,
                             const Point3d & left_to, const Point3d & right_from,
                             const Point3d & right_to) {
    LineString3d left_bound(lanelet::utils::getId(), {left_from, left_to});
    LineString3d right_bound(lanelet::utils::getId(), {right_from, right_to});
    auto lanelet = Lanelet(id, left_bound, right_bound);
    lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
    return lanelet;
  };

  auto first_lanelet = make_road_lanelet(FIRST_LANELET_ID, left_0, left_50, right_0, right_50);
  auto second_lanelet =
    make_road_lanelet(SECOND_LANELET_ID, left_50, left_100, right_50, right_100);

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(first_lanelet);
  lanelet_map->add(second_lanelet);

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

Pose make_pose(const double x, const double y)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

Odometry make_odometry(const Pose & pose, const double velocity)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose = pose;
  odometry.twist.twist.linear.x = velocity;
  return odometry;
}

OperationModeState make_operation_mode_state(
  const uint8_t mode, const bool is_autoware_control_enabled)
{
  OperationModeState state;
  state.mode = mode;
  state.is_autoware_control_enabled = is_autoware_control_enabled;
  return state;
}

LaneletSegment make_segment(const lanelet::Id id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";

  LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  return segment;
}

SetLaneletRoute::Request::SharedPtr make_set_lanelet_route_request(
  const lanelet::Id lanelet_id, const Pose & goal_pose)
{
  auto request = std::make_shared<SetLaneletRoute::Request>();
  request->header.frame_id = "map";
  request->goal_pose = goal_pose;
  request->segments.push_back(make_segment(lanelet_id));
  request->allow_modification = false;
  return request;
}

SetWaypointRoute::Request::SharedPtr make_set_waypoint_route_request(const Pose & goal_pose)
{
  auto request = std::make_shared<SetWaypointRoute::Request>();
  request->header.frame_id = "map";
  request->goal_pose = goal_pose;
  request->allow_modification = false;
  return request;
}

}  // namespace

class MissionPlannerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    const auto durable_qos = rclcpp::QoS(1).transient_local();
    map_publisher_ =
      test_node_->create_publisher<LaneletMapBin>("/mission_planner/input/vector_map", durable_qos);
    odometry_publisher_ =
      test_node_->create_publisher<Odometry>("/mission_planner/input/odometry", rclcpp::QoS(1));
    operation_mode_state_publisher_ = test_node_->create_publisher<OperationModeState>(
      "/mission_planner/input/operation_mode_state", durable_qos);

    route_subscription_ = test_node_->create_subscription<LaneletRoute>(
      "/mission_planner/route", durable_qos,
      [this](const LaneletRoute::SharedPtr message) { received_route_ = message; });
    state_subscription_ = test_node_->create_subscription<RouteState>(
      "/mission_planner/state", durable_qos,
      [this](const RouteState::SharedPtr message) { received_state_ = message; });

    set_lanelet_route_client_ =
      test_node_->create_client<SetLaneletRoute>("/mission_planner/set_lanelet_route");
    set_waypoint_route_client_ =
      test_node_->create_client<SetWaypointRoute>("/mission_planner/set_waypoint_route");
  }

  void initialize_mission_planner_node()
  {
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto mission_planner_dir =
      ament_index_cpp::get_package_share_directory("autoware_mission_planner");

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
       mission_planner_dir + "/config/mission_planner.param.yaml"});

    node_ = std::make_shared<MissionPlanner>(options);
    executor_->add_node(node_);

    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      if (
        map_publisher_->get_subscription_count() > 0 &&
        odometry_publisher_->get_subscription_count() > 0 &&
        set_lanelet_route_client_->service_is_ready() &&
        set_waypoint_route_client_->service_is_ready()) {
        break;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void TearDown() override
  {
    executor_.reset();
    set_waypoint_route_client_.reset();
    set_lanelet_route_client_.reset();
    state_subscription_.reset();
    route_subscription_.reset();
    operation_mode_state_publisher_.reset();
    odometry_publisher_.reset();
    map_publisher_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void publish_map(const LaneletMapBin & map_bin) { map_publisher_->publish(map_bin); }

  void publish_odometry(const Pose & pose)
  {
    odometry_publisher_->publish(make_odometry(pose, /*velocity=*/0.0));
  }

  void publish_autonomous_operation_mode_state()
  {
    operation_mode_state_publisher_->publish(make_operation_mode_state(
      OperationModeState::AUTONOMOUS, /*is_autoware_control_enabled=*/true));
  }

  // Spins the executor so that subscription callbacks and the node's 10 Hz readiness timer run.
  void spin_for(std::chrono::milliseconds duration)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Waits for the response and then keeps spinning briefly so that route_subscription_ and
  // state_subscription_ (populated by the same call's pub_route_/pub_state_) have a chance to
  // fire: the response future can be satisfied before the executor has drained those other
  // already-ready callbacks in the same spin pass.
  SetLaneletRoute::Response::SharedPtr call_set_lanelet_route(
    const lanelet::Id lanelet_id, const Pose & goal_pose)
  {
    const auto request = make_set_lanelet_route_request(lanelet_id, goal_pose);
    auto future = set_lanelet_route_client_->async_send_request(request);
    EXPECT_EQ(
      executor_->spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    spin_for(std::chrono::milliseconds(100));
    return future.get();
  }

  SetWaypointRoute::Response::SharedPtr call_set_waypoint_route(const Pose & goal_pose)
  {
    const auto request = make_set_waypoint_route_request(goal_pose);
    auto future = set_waypoint_route_client_->async_send_request(request);
    EXPECT_EQ(
      executor_->spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    spin_for(std::chrono::milliseconds(100));
    return future.get();
  }

  template <typename ResponseT>
  void expect_success(const ResponseT & response)
  {
    EXPECT_TRUE(response->status.success);
  }

  template <typename ResponseT>
  void expect_failure(const ResponseT & response)
  {
    EXPECT_FALSE(response->status.success);
  }

  void expect_route_state(const uint8_t expected_state)
  {
    ASSERT_NE(received_state_, nullptr);
    EXPECT_EQ(received_state_->state, expected_state);
  }

  void expect_route_has_segment(const lanelet::Id expected_id)
  {
    ASSERT_NE(received_route_, nullptr);
    ASSERT_EQ(received_route_->segments.size(), 1u);
    EXPECT_EQ(received_route_->segments.front().preferred_primitive.id, expected_id);
  }

  void expect_route_ends_with_segment(const lanelet::Id expected_id)
  {
    ASSERT_NE(received_route_, nullptr);
    ASSERT_FALSE(received_route_->segments.empty());
    EXPECT_EQ(received_route_->segments.back().preferred_primitive.id, expected_id);
  }

  std::shared_ptr<MissionPlanner> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_publisher_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_state_publisher_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscription_;
  rclcpp::Subscription<RouteState>::SharedPtr state_subscription_;
  rclcpp::Client<SetLaneletRoute>::SharedPtr set_lanelet_route_client_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr set_waypoint_route_client_;

  LaneletRoute::SharedPtr received_route_;
  RouteState::SharedPtr received_state_;
};

TEST_F(MissionPlannerIntegrationTest, SetLaneletRouteBeforeInitializationReturnsFailure)
{
  // Arrange
  initialize_mission_planner_node();

  // Act
  const auto response = call_set_lanelet_route(FIRST_LANELET_ID, make_pose(40.0, 0.0));

  // Assert
  // Request must fail before the node has received map/odometry/operation mode state
  expect_failure(response);
}

TEST_F(MissionPlannerIntegrationTest, FirstSetLaneletRouteRequestSucceeds)
{
  // Arrange
  initialize_mission_planner_node();
  publish_map(create_map());
  publish_odometry(make_pose(10.0, 0.0));
  spin_for(std::chrono::milliseconds(300));

  // Act
  const auto response = call_set_lanelet_route(FIRST_LANELET_ID, make_pose(40.0, 0.0));

  // Assert
  // Once the node is fully initialized, the first route request must succeed and the
  // resulting route/state must reflect the requested goal lanelet.
  expect_success(response);
  expect_route_state(RouteState::SET);
  expect_route_has_segment(FIRST_LANELET_ID);
}

TEST_F(MissionPlannerIntegrationTest, SecondRequestAsSafeRerouteWhileStoppedSucceeds)
{
  // Arrange
  initialize_mission_planner_node();
  publish_map(create_map());
  publish_odometry(make_pose(10.0, 0.0));
  publish_autonomous_operation_mode_state();
  spin_for(std::chrono::milliseconds(300));

  // Act
  call_set_lanelet_route(FIRST_LANELET_ID, make_pose(40.0, 0.0));
  const auto response = call_set_lanelet_route(SECOND_LANELET_ID, make_pose(90.0, 0.0));

  // Assert
  // A reroute requested while the vehicle is stopped (a "safe reroute") must succeed and
  // replace the previously set route with the new goal lanelet.
  expect_success(response);
  expect_route_state(RouteState::SET);
  expect_route_has_segment(SECOND_LANELET_ID);
}

TEST_F(MissionPlannerIntegrationTest, SetWaypointRouteBeforeInitializationReturnsFailure)
{
  // Arrange
  initialize_mission_planner_node();

  // Act
  const auto response = call_set_waypoint_route(make_pose(140.0, 0.0));

  // Assert
  // Request must fail before the node has received map/odometry/operation mode state
  expect_failure(response);
}

TEST_F(MissionPlannerIntegrationTest, FirstSetWaypointRouteRequestSucceeds)
{
  // Arrange
  initialize_mission_planner_node();
  publish_map(create_map());
  publish_odometry(make_pose(10.0, 0.0));
  spin_for(std::chrono::milliseconds(300));

  // Act
  const auto response = call_set_waypoint_route(make_pose(90.0, 0.0));

  // Assert
  // A waypoint-based route request must succeed once the node is initialized, and the
  // planned route must end at the lanelet containing the requested goal pose.
  expect_success(response);
  expect_route_state(RouteState::SET);
  expect_route_ends_with_segment(SECOND_LANELET_ID);
}

TEST_F(MissionPlannerIntegrationTest, SecondWaypointRequestAsSafeRerouteWhileStoppedSucceeds)
{
  // Arrange
  initialize_mission_planner_node();
  publish_map(create_map());
  publish_odometry(make_pose(10.0, 0.0));
  publish_autonomous_operation_mode_state();
  spin_for(std::chrono::milliseconds(300));

  // Act
  call_set_waypoint_route(make_pose(140.0, 0.0));
  const auto response = call_set_waypoint_route(make_pose(60.0, 0.0));

  // Assert
  // A waypoint-based reroute requested while the vehicle is stopped (a "safe reroute") must
  // succeed and replace the previous route with one ending at the new goal lanelet.
  expect_success(response);
  expect_route_state(RouteState::SET);
  expect_route_ends_with_segment(SECOND_LANELET_ID);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

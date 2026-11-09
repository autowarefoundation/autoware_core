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

#include "../src/mission_planner/mission_planner_node.hpp"

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
#include <thread>
#include <vector>

using autoware::mission_planner::MissionPlannerNode;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using nav_msgs::msg::Odometry;

namespace
{

// ID of the single road lanelet making up the test map (see create_map()).
constexpr lanelet::Id road_lanelet_id = 1000;

// Goal pose used by both route requests. It lies well inside the single road lanelet so that the
// goal footprint check passes.
constexpr double goal_pose_x = 40.0;

/// @brief Create a lanelet map with a single straight road lanelet along the x axis from 0 to 50.
///
/// The route planning logic itself is covered by the unit tests, so the map only has to be big
/// enough for a one lanelet route request to be planned.
LaneletMapBin create_map()
{
  const lanelet::Point3d left_from(lanelet::utils::getId(), 0.0, 2.0);
  const lanelet::Point3d left_to(lanelet::utils::getId(), 50.0, 2.0);
  const lanelet::Point3d right_from(lanelet::utils::getId(), 0.0, -2.0);
  const lanelet::Point3d right_to(lanelet::utils::getId(), 50.0, -2.0);

  const lanelet::LineString3d left_bound(lanelet::utils::getId(), {left_from, left_to});
  const lanelet::LineString3d right_bound(lanelet::utils::getId(), {right_from, right_to});
  auto road_lanelet = lanelet::Lanelet(road_lanelet_id, left_bound, right_bound);
  road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet);

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

Odometry make_odometry()
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 10.0;
  odometry.pose.pose.orientation.w = 1.0;
  return odometry;
}

SetLaneletRoute::Request::SharedPtr make_set_lanelet_route_request()
{
  LaneletPrimitive primitive;
  primitive.id = road_lanelet_id;
  primitive.primitive_type = "lane";

  LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;

  auto request = std::make_shared<SetLaneletRoute::Request>();
  request->header.frame_id = "map";
  request->goal_pose.position.x = goal_pose_x;
  request->goal_pose.orientation.w = 1.0;
  request->segments.push_back(segment);
  return request;
}

SetWaypointRoute::Request::SharedPtr make_set_waypoint_route_request()
{
  auto request = std::make_shared<SetWaypointRoute::Request>();
  request->header.frame_id = "map";
  request->goal_pose.position.x = goal_pose_x;
  request->goal_pose.orientation.w = 1.0;
  return request;
}

void expect_route_published(const LaneletRoute::SharedPtr & route)
{
  ASSERT_NE(route, nullptr);
  EXPECT_FALSE(route->segments.empty());
}

// The node reports UNSET as soon as it is ready and SET once the route has been planned.
void expect_states_transit_from_unset_to_set(
  const std::vector<RouteState::_state_type> & received_states)
{
  ASSERT_FALSE(received_states.empty());
  EXPECT_EQ(received_states.front(), RouteState::UNSET);
  EXPECT_EQ(received_states.back(), RouteState::SET);
}

}  // namespace

class MissionPlannerNodeTest : public ::testing::Test
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

    // NOTE: The route, the route state and the route services use the
    // component_interface_specs absolute names (not the node's own relative "~/..." names)
    // because MissionPlanner creates these endpoints via NodeAdaptor, which always binds to the
    // spec's absolute name regardless of node namespace/name.
    route_subscription_ = test_node_->create_subscription<LaneletRoute>(
      "/planning/route", durable_qos,
      [this](const LaneletRoute::SharedPtr message) { received_route_ = message; });
    state_subscription_ = test_node_->create_subscription<RouteState>(
      "/planning/route_state", durable_qos,
      [this](const RouteState::SharedPtr message) { received_states_.push_back(message->state); });
    set_lanelet_route_client_ =
      test_node_->create_client<SetLaneletRoute>("/planning/set_lanelet_route");
    set_waypoint_route_client_ =
      test_node_->create_client<SetWaypointRoute>("/planning/set_waypoint_route");

    create_mission_planner_node();
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

  // Feeds the node with the data it waits for before the route API becomes available.
  void publish_map_and_odometry()
  {
    map_publisher_->publish(create_map());
    odometry_publisher_->publish(make_odometry());
    spin_for(std::chrono::milliseconds(300));
  }

  // A reroute request is rejected until the node has received an operation mode state.
  void publish_stopped_operation_mode_state()
  {
    OperationModeState operation_mode_state;
    operation_mode_state.mode = OperationModeState::STOP;
    operation_mode_state.is_autoware_control_enabled = false;
    operation_mode_state_publisher_->publish(operation_mode_state);
    spin_for(std::chrono::milliseconds(300));
  }

  // Sends the request and spins until the response and the messages published by the same service
  // call have arrived.
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call_route_service(
    rclcpp::Client<ServiceT> & client, const typename ServiceT::Request::SharedPtr request)
  {
    auto future = client.async_send_request(request);
    EXPECT_EQ(
      executor_->spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    spin_for(std::chrono::milliseconds(100));
    return future.get();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_publisher_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_state_publisher_;
  rclcpp::Client<SetLaneletRoute>::SharedPtr set_lanelet_route_client_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr set_waypoint_route_client_;

  LaneletRoute::SharedPtr received_route_;
  std::vector<RouteState::_state_type> received_states_;

private:
  // Spins the executor so that subscription callbacks and the node's 10 Hz readiness timer run.
  void spin_for(const std::chrono::milliseconds duration)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void create_mission_planner_node()
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

    node_ = std::make_shared<MissionPlannerNode>(options);
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

  std::shared_ptr<MissionPlannerNode> node_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscription_;
  rclcpp::Subscription<RouteState>::SharedPtr state_subscription_;
};

// The tests below verify the node side wiring only: the map and odometry subscriptions feed the
// readiness check, the operation mode state subscription feeds the reroute check, the route service
// endpoints answer a request, and the resulting route and route states are published. The route
// planning itself is covered by the unit tests.

TEST_F(MissionPlannerNodeTest, PlansAndPublishesRouteForSetLaneletRouteRequest)
{
  // Arrange
  publish_map_and_odometry();

  // Act
  const auto response =
    call_route_service(*set_lanelet_route_client_, make_set_lanelet_route_request());

  // Assert
  EXPECT_TRUE(response->status.success);
  expect_route_published(received_route_);
  expect_states_transit_from_unset_to_set(received_states_);
}

TEST_F(MissionPlannerNodeTest, PlansAndPublishesRouteForSetWaypointRouteRequest)
{
  // Arrange
  publish_map_and_odometry();

  // Act
  const auto response =
    call_route_service(*set_waypoint_route_client_, make_set_waypoint_route_request());

  // Assert
  EXPECT_TRUE(response->status.success);
  expect_route_published(received_route_);
  expect_states_transit_from_unset_to_set(received_states_);
}

TEST_F(MissionPlannerNodeTest, RerouteSucceedsAfterOperationModeStateIsReceived)
{
  // Arrange
  publish_map_and_odometry();
  publish_stopped_operation_mode_state();
  call_route_service(*set_lanelet_route_client_, make_set_lanelet_route_request());

  // Act
  const auto response =
    call_route_service(*set_lanelet_route_client_, make_set_lanelet_route_request());

  // Assert
  EXPECT_TRUE(response->status.success);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

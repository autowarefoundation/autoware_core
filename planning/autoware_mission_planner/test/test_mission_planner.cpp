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

#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::mission_planner::MissionPlanner;
using autoware::mission_planner::MissionPlannerConfig;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;

using SetRouteResponse = autoware_adapi_v1_msgs::srv::SetRoute::Response;
using SetRoutePointsResponse = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;

// IDs of the two colinear road lanelets making up the test map (see create_map()).
constexpr lanelet::Id first_lanelet_id = 1000;
constexpr lanelet::Id second_lanelet_id = 1001;

constexpr double map_frame_transform_x = 30.0;
constexpr char non_map_frame[] = "sensor_frame";

constexpr double reroute_time_threshold = 10.0;
constexpr double minimum_reroute_length = 30.0;
constexpr double arrival_check_duration = 1.0;

// x of the pose where the vehicle starts in every test (see start_odometry()).
constexpr double start_x = 10.0;

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

  auto road_lanelet = [](
                        const lanelet::Id id, const Point3d & left_from, const Point3d & left_to,
                        const Point3d & right_from, const Point3d & right_to) {
    LineString3d left_bound(lanelet::utils::getId(), {left_from, left_to});
    LineString3d right_bound(lanelet::utils::getId(), {right_from, right_to});
    auto lanelet = Lanelet(id, left_bound, right_bound);
    lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
    return lanelet;
  };

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet(first_lanelet_id, left_0, left_50, right_0, right_50));
  lanelet_map->add(road_lanelet(second_lanelet_id, left_50, left_100, right_50, right_100));

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

// NOTE: values below mirror autoware_test_utils/config/test_vehicle_info.param.yaml
autoware::vehicle_info_utils::VehicleInfo vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    /* wheel_radius_m= */ 0.383, /* wheel_width_m= */ 0.235, /* wheel_base_m= */ 2.79,
    /* wheel_tread_m= */ 1.64, /* front_overhang_m= */ 1.0, /* rear_overhang_m= */ 1.1,
    /* left_overhang_m= */ 0.128, /* right_overhang_m= */ 0.128, /* vehicle_height_m= */ 2.5,
    /* max_steer_angle_rad= */ 0.70);
}

// NOTE: values below mirror autoware_mission_planner/config/mission_planner.param.yaml
MissionPlannerConfig default_config()
{
  MissionPlannerConfig config;
  config.map_frame = "map";
  config.reroute_time_threshold = reroute_time_threshold;
  config.minimum_reroute_length = minimum_reroute_length;
  config.allow_reroute_in_autonomous_mode = true;
  config.arrival_checker_threshold.distance = 1.0;
  config.arrival_checker_threshold.angle = 45.0 * M_PI / 180.0;
  config.arrival_checker_threshold.duration = arrival_check_duration;
  config.default_planner_parameters.goal_angle_threshold_deg = 45.0;
  config.default_planner_parameters.enable_correct_goal_pose = false;
  config.default_planner_parameters.consider_no_drivable_lanes = false;
  config.default_planner_parameters.check_footprint_inside_lanes = true;
  config.vehicle_info = vehicle_info();
  return config;
}

Pose pose(const double x, const double y = 0.0)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

Odometry::ConstSharedPtr odometry(
  const Pose & pose, const double velocity = 0.0, const double time_sec = 0.0)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = rclcpp::Time(static_cast<int64_t>(time_sec * 1e9));
  odometry.pose.pose = pose;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<Odometry>(odometry);
}

// Odometry of the vehicle at start_x moving at `velocity`.
Odometry::ConstSharedPtr start_odometry(const double velocity = 0.0)
{
  return odometry(pose(start_x), velocity);
}

OperationModeState::ConstSharedPtr operation_mode_state(
  const uint8_t mode, const bool is_autoware_control_enabled)
{
  OperationModeState state;
  state.mode = mode;
  state.is_autoware_control_enabled = is_autoware_control_enabled;
  return std::make_shared<OperationModeState>(state);
}

LaneletSegment segment(const lanelet::Id id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";

  LaneletSegment segment;
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  return segment;
}

SetLaneletRoute::Request lanelet_route_request(
  const std::vector<lanelet::Id> & lanelet_ids, const Pose & goal_pose,
  const std::string & frame_id = "map")
{
  SetLaneletRoute::Request request;
  request.header.frame_id = frame_id;
  request.goal_pose = goal_pose;
  for (const auto lanelet_id : lanelet_ids) {
    request.segments.push_back(segment(lanelet_id));
  }
  return request;
}

SetWaypointRoute::Request waypoint_route_request(
  const Pose & goal_pose, const std::vector<Pose> & waypoints = {},
  const std::string & frame_id = "map")
{
  SetWaypointRoute::Request request;
  request.header.frame_id = frame_id;
  request.goal_pose = goal_pose;
  request.waypoints = waypoints;
  return request;
}

// Static transform from the non-map request frame into the map frame, shifting poses by
// map_frame_transform_x along x.
TransformStamped transform_to_map()
{
  TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = non_map_frame;
  transform.transform.translation.x = map_frame_transform_x;
  transform.transform.rotation.w = 1.0;
  return transform;
}

// Feeds odometry of the vehicle standing still at `pose` every 0.1 s for `duration_sec` seconds.
void stay_stopped_at(MissionPlanner & mission_planner, const Pose & pose, const double duration_sec)
{
  for (double time_sec = 0.0; time_sec <= duration_sec; time_sec += 0.1) {
    mission_planner.on_odometry(odometry(pose, 0.0, time_sec));
  }
}

// Returns a state change callback that appends every notified state to `states`. `states` must
// outlive the mission planner that holds the callback.
MissionPlanner::ChangeStateCallback record_states(std::vector<RouteState::_state_type> & states)
{
  return [&states](const auto state) { states.push_back(state); };
}

template <typename Result>
void expect_success_response(const Result & result)
{
  EXPECT_TRUE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, 0);
}

template <typename Result>
void expect_fail_response_with_code(const Result & result, const uint16_t code)
{
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, code);
  EXPECT_FALSE(result.route.has_value());
}

// Owns the objects that a mission planner refers to, so that the mission planners created by the
// fixture stay valid for the whole test. Every state change notified by these mission planners is
// appended to `states`.
class MissionPlannerTest : public ::testing::Test
{
protected:
  // Creates a mission planner that has received neither a map nor odometry.
  MissionPlanner create_mission_planner(const MissionPlannerConfig & config = default_config())
  {
    return MissionPlanner(config, tf_buffer, record_states(states));
  }

  // Creates a mission planner in the ready state (RouteState::UNSET) by feeding the map and the
  // odometry of the vehicle stopped at start_x. The states notified during the initialization are
  // discarded.
  MissionPlanner create_initialized_mission_planner(
    const MissionPlannerConfig & config = default_config())
  {
    auto mission_planner = create_mission_planner(config);
    mission_planner.on_map(std::make_shared<LaneletMapBin>(create_map()));
    mission_planner.on_odometry(start_odometry());
    mission_planner.check_initialization();
    states.clear();
    return mission_planner;
  }

  // Expects that the notified states are exactly `expected_states`, in this order.
  void expect_states_transition(const std::vector<RouteState::_state_type> & expected_states) const
  {
    EXPECT_EQ(states, expected_states);
  }

  tf2::BufferCore tf_buffer;
  std::vector<RouteState::_state_type> states;
};

}  // namespace

TEST_F(MissionPlannerTest, CheckInitializationFailsWithoutMapAndOdometry)
{
  // Arrange
  auto mission_planner = create_mission_planner();

  // Act
  const auto is_initialized = mission_planner.check_initialization();

  // Assert
  EXPECT_FALSE(is_initialized);
}

TEST_F(MissionPlannerTest, CheckInitializationFailsWithoutOdometry)
{
  // Arrange
  auto mission_planner = create_mission_planner();
  mission_planner.on_map(std::make_shared<LaneletMapBin>(create_map()));

  // Act
  const auto is_initialized = mission_planner.check_initialization();

  // Assert
  EXPECT_FALSE(is_initialized);
}

TEST_F(MissionPlannerTest, CheckInitializationFailsWithoutMap)
{
  // Arrange
  auto mission_planner = create_mission_planner();
  mission_planner.on_odometry(start_odometry());

  // Act
  const auto is_initialized = mission_planner.check_initialization();

  // Assert
  EXPECT_FALSE(is_initialized);
}

TEST_F(MissionPlannerTest, CheckInitializationChangesStateToUnset)
{
  // Arrange
  auto mission_planner = create_mission_planner();
  mission_planner.on_map(std::make_shared<LaneletMapBin>(create_map()));
  mission_planner.on_odometry(start_odometry());

  // Act
  const auto is_initialized = mission_planner.check_initialization();

  // Assert
  EXPECT_TRUE(is_initialized);
  expect_states_transition({RouteState::UNSET});
}

TEST_F(MissionPlannerTest, ClearRouteBeforeInitializationHasNoEffect)
{
  // Arrange
  auto mission_planner = create_mission_planner();

  // Act
  const auto response = mission_planner.clear_route();

  // Assert
  // Unlike the other responses, this one is a success with a non-zero code (NO_EFFECT), so it is
  // the only response that is checked without expect_success_response().
  EXPECT_TRUE(response.status.success);
  EXPECT_EQ(response.status.code, ResponseStatus::NO_EFFECT);
  EXPECT_TRUE(states.empty());
}

TEST_F(MissionPlannerTest, ClearRouteAfterInitializationChangesStateToUnset)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto response = mission_planner.clear_route();

  // Assert
  EXPECT_TRUE(response.status.success);
  expect_states_transition({RouteState::UNSET});
}

TEST_F(MissionPlannerTest, SetLaneletRouteBeforeInitializationFailsWithInvalidState)
{
  // Arrange
  const auto request = lanelet_route_request({}, pose(40.0));

  auto mission_planner = create_mission_planner();

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, SetWaypointRouteBeforeInitializationFailsWithInvalidState)
{
  // Arrange
  const auto request = waypoint_route_request(pose(90.0));

  auto mission_planner = create_mission_planner();

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  expect_fail_response_with_code(result, SetRoutePointsResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, SetLaneletRouteWithoutSegmentsFailsAndRestoresUnsetState)
{
  // Arrange
  const auto request = lanelet_route_request({}, pose(40.0));

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_PLANNER_FAILED);
  expect_states_transition({RouteState::ROUTING, RouteState::UNSET});
}

TEST_F(MissionPlannerTest, SetLaneletRouteFailsWhenTransformToMapIsUnavailable)
{
  // Arrange
  // The transform of the request frame is never registered in tf_buffer.
  const auto request = lanelet_route_request({}, pose(10.0), non_map_frame);

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  expect_fail_response_with_code(result, ResponseStatus::TRANSFORM_ERROR);
  // NOTE: This pins the current behavior, which is probably a bug. Unlike the other failure paths,
  // the state is not restored to UNSET, so the mission planner stays in ROUTING and rejects the
  // following requests with ERROR_INVALID_STATE until the route is cleared. Once fixed, expect
  // {ROUTING, UNSET} (or no state change at all) instead.
  expect_states_transition({RouteState::ROUTING});
}

TEST_F(MissionPlannerTest, SetWaypointRouteFailsWhenTransformToMapIsUnavailable)
{
  // Arrange
  // The transform of the request frame is never registered in tf_buffer.
  const auto request = waypoint_route_request(pose(60.0), {}, non_map_frame);

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  expect_fail_response_with_code(result, ResponseStatus::TRANSFORM_ERROR);
  // NOTE: This pins the current behavior, which is probably a bug. See
  // SetLaneletRouteFailsWhenTransformToMapIsUnavailable.
  expect_states_transition({RouteState::ROUTING});
}

TEST_F(MissionPlannerTest, SetLaneletRouteSucceedsAfterInitialization)
{
  // Arrange
  const auto request = lanelet_route_request({first_lanelet_id}, pose(40.0));

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  ASSERT_EQ(result.route->segments.size(), 1U);
  EXPECT_EQ(result.route->segments.front().preferred_primitive.id, first_lanelet_id);
  EXPECT_EQ(result.route->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.route->start_pose.position.x, start_x);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 40.0);
  EXPECT_DOUBLE_EQ(result.initial_pose.position.x, start_x);
  EXPECT_TRUE(result.route_marker.has_value());
  expect_states_transition({RouteState::ROUTING, RouteState::SET});
}

TEST_F(MissionPlannerTest, SetLaneletRouteTransformsGoalPoseIntoMapFrame)
{
  // Arrange
  tf_buffer.setTransform(transform_to_map(), "test", true);
  const auto request = lanelet_route_request({first_lanelet_id}, pose(10.0), non_map_frame);

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 10.0 + map_frame_transform_x);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteFailsWhenOperationModeStateIsNotReceived)
{
  // Arrange
  const auto first_request = lanelet_route_request({first_lanelet_id}, pose(40.0));
  const auto second_request = lanelet_route_request({}, pose(90.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(first_request);

  // Act
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_PLANNER_UNREADY);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteFailsWhenNotAllowedInAutonomousMode)
{
  // Arrange
  auto config = default_config();
  config.allow_reroute_in_autonomous_mode = false;
  const auto autonomous_mode = operation_mode_state(OperationModeState::AUTONOMOUS, true);
  const auto first_request = lanelet_route_request({first_lanelet_id}, pose(40.0));
  const auto second_request = lanelet_route_request({}, pose(90.0));

  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_operation_mode_state(autonomous_mode);
  mission_planner.set_lanelet_route(first_request);

  // Act
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteSucceedsWhenNotInAutonomousMode)
{
  // Arrange
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const auto odom_while_driving = start_odometry(5.0);
  const auto stop_mode_state = operation_mode_state(OperationModeState::STOP, false);
  const auto first_request = lanelet_route_request({first_lanelet_id}, pose(40.0));
  const auto second_request = lanelet_route_request({first_lanelet_id}, pose(20.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.on_odometry(odom_while_driving);
  mission_planner.on_operation_mode_state(stop_mode_state);
  mission_planner.set_lanelet_route(first_request);

  // Act
  // The reroute safety check is skipped outside autonomous mode, so even a short new route is
  // accepted while driving.
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteSucceedsWhenAutowareControlIsDisabled)
{
  // Arrange
  auto config = default_config();
  config.allow_reroute_in_autonomous_mode = false;
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const auto odom_while_driving = start_odometry(5.0);
  const auto autoware_control_disabled_state =
    operation_mode_state(OperationModeState::AUTONOMOUS, false);
  const auto first_request = lanelet_route_request({first_lanelet_id}, pose(40.0));
  const auto second_request = lanelet_route_request({first_lanelet_id}, pose(20.0));

  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_odometry(odom_while_driving);
  mission_planner.on_operation_mode_state(autoware_control_disabled_state);
  mission_planner.set_lanelet_route(first_request);

  // Act
  // The vehicle is not driven by Autoware, so it is not treated as autonomous driving and the
  // reroute safety check is skipped.
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetLaneletRouteTwiceWhileStoppedInAutonomousModeReroutesToSecondRoute)
{
  // Arrange
  const auto autonomous_mode = operation_mode_state(OperationModeState::AUTONOMOUS, true);
  const auto second_goal_pose = pose(90.0);
  const auto first_request = lanelet_route_request({first_lanelet_id}, pose(40.0));
  const auto second_request =
    lanelet_route_request({first_lanelet_id, second_lanelet_id}, second_goal_pose);

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.on_operation_mode_state(autonomous_mode);

  // Act
  // The vehicle is stopped, so the reroute safety check passes regardless of the route length.
  const auto first_result = mission_planner.set_lanelet_route(first_request);
  const auto second_result = mission_planner.set_lanelet_route(second_request);
  // The second route is in effect only if the vehicle arrives at its goal.
  stay_stopped_at(mission_planner, second_goal_pose, arrival_check_duration + 0.5);

  // Assert
  expect_success_response(first_result);
  expect_success_response(second_result);
  ASSERT_TRUE(second_result.route.has_value());
  EXPECT_EQ(second_result.route->segments.size(), 2U);
  expect_states_transition(
    {RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET,
     RouteState::ARRIVED});
}

TEST_F(MissionPlannerTest, SetLaneletRouteTwiceWhileDrivingKeepsFirstRouteWhenRerouteIsUnsafe)
{
  // Arrange
  // Driving fast enough that the required safety length (velocity * reroute_time_threshold = 100 m)
  // exceeds the 30 m shared with the new route.
  const auto odom_with_high_velocity = start_odometry(10.0);
  const auto autonomous_mode = operation_mode_state(OperationModeState::AUTONOMOUS, true);
  const auto first_goal_pose = pose(90.0);
  const auto first_request =
    lanelet_route_request({first_lanelet_id, second_lanelet_id}, first_goal_pose);
  const auto second_request = lanelet_route_request({first_lanelet_id}, pose(40.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.on_odometry(odom_with_high_velocity);
  mission_planner.on_operation_mode_state(autonomous_mode);

  // Act
  const auto first_result = mission_planner.set_lanelet_route(first_request);
  const auto second_result = mission_planner.set_lanelet_route(second_request);
  // The first route stays in effect only if the vehicle arrives at its goal.
  stay_stopped_at(mission_planner, first_goal_pose, arrival_check_duration + 0.5);

  // Assert
  expect_success_response(first_result);
  expect_fail_response_with_code(second_result, SetRouteResponse::ERROR_REROUTE_FAILED);
  expect_states_transition(
    {RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET,
     RouteState::ARRIVED});
}

TEST_F(
  MissionPlannerTest, SetLaneletRouteRerouteFailsWhenSharedRouteIsShorterThanMinimumRerouteLength)
{
  // Arrange
  auto config = default_config();
  config.minimum_reroute_length = 40.0;
  // Driving slowly, so the velocity-dependent safety length (1 m/s * 10 s = 10 m) is shorter than
  // the 30 m shared with the new route and only minimum_reroute_length can reject the reroute.
  const auto odom_with_low_velocity = start_odometry(1.0);
  const auto autonomous_mode = operation_mode_state(OperationModeState::AUTONOMOUS, true);
  const auto first_goal_pose = pose(90.0);
  const auto first_request =
    lanelet_route_request({first_lanelet_id, second_lanelet_id}, first_goal_pose);
  const auto second_request = lanelet_route_request({first_lanelet_id}, pose(40.0));

  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_odometry(odom_with_low_velocity);
  mission_planner.on_operation_mode_state(autonomous_mode);
  mission_planner.set_lanelet_route(first_request);

  // Act
  const auto result = mission_planner.set_lanelet_route(second_request);
  // The first route stays in effect only if the vehicle arrives at its goal.
  stay_stopped_at(mission_planner, first_goal_pose, arrival_check_duration + 0.5);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_REROUTE_FAILED);
  expect_states_transition(
    {RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET,
     RouteState::ARRIVED});
}

TEST_F(MissionPlannerTest, SetWaypointRouteRerouteFailsWhenOperationModeStateIsNotReceived)
{
  // Arrange
  const auto first_request = waypoint_route_request(pose(90.0));
  const auto second_request = waypoint_route_request(pose(40.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_waypoint_route(first_request);

  // Act
  const auto result = mission_planner.set_waypoint_route(second_request);

  // Assert
  expect_fail_response_with_code(result, SetRoutePointsResponse::ERROR_PLANNER_UNREADY);
}

TEST_F(MissionPlannerTest, SetWaypointRouteTwiceWhileDrivingKeepsFirstRouteWhenRerouteIsUnsafe)
{
  // Arrange
  // Driving fast enough that the required safety length (velocity * reroute_time_threshold = 100 m)
  // exceeds the 30 m shared with the new route.
  const auto odom_with_high_velocity = start_odometry(10.0);
  const auto autonomous_mode = operation_mode_state(OperationModeState::AUTONOMOUS, true);
  const auto first_goal_pose = pose(90.0);
  const auto first_request = waypoint_route_request(first_goal_pose);
  const auto second_request = waypoint_route_request(pose(40.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.on_odometry(odom_with_high_velocity);
  mission_planner.on_operation_mode_state(autonomous_mode);

  // Act
  const auto first_result = mission_planner.set_waypoint_route(first_request);
  const auto second_result = mission_planner.set_waypoint_route(second_request);
  // The first route stays in effect only if the vehicle arrives at its goal.
  stay_stopped_at(mission_planner, first_goal_pose, arrival_check_duration + 0.5);

  // Assert
  expect_success_response(first_result);
  expect_fail_response_with_code(second_result, SetRoutePointsResponse::ERROR_REROUTE_FAILED);
  expect_states_transition(
    {RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET,
     RouteState::ARRIVED});
}

TEST_F(MissionPlannerTest, SetWaypointRouteRerouteSucceedsWhenNotInAutonomousMode)
{
  // Arrange
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const auto odom_while_driving = start_odometry(5.0);
  const auto stop_mode_state = operation_mode_state(OperationModeState::STOP, false);
  const auto first_request = waypoint_route_request(pose(40.0));
  const auto second_request = waypoint_route_request(pose(20.0));

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.on_odometry(odom_while_driving);
  mission_planner.on_operation_mode_state(stop_mode_state);
  mission_planner.set_waypoint_route(first_request);

  // Act
  // The reroute safety check is skipped outside autonomous mode, so even a short new route is
  // accepted while driving.
  const auto result = mission_planner.set_waypoint_route(second_request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetWaypointRouteRerouteSucceedsWhenAutowareControlIsDisabled)
{
  // Arrange
  // Rerouting while driving autonomously is disallowed, so only the disabled Autoware control can
  // make the reroute succeed.
  auto config = default_config();
  config.allow_reroute_in_autonomous_mode = false;
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const auto odom_while_driving = start_odometry(5.0);
  const auto autoware_control_disabled_state =
    operation_mode_state(OperationModeState::AUTONOMOUS, false);
  const auto first_request = waypoint_route_request(pose(40.0));
  const auto second_request = waypoint_route_request(pose(20.0));

  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_odometry(odom_while_driving);
  mission_planner.on_operation_mode_state(autoware_control_disabled_state);
  mission_planner.set_waypoint_route(first_request);

  // Act
  // The vehicle is not driven by Autoware, so it is not treated as autonomous driving and the
  // reroute safety check is skipped.
  const auto result = mission_planner.set_waypoint_route(second_request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetWaypointRouteFailsWhenGoalIsOutsideTheMap)
{
  // Arrange
  const auto pose_outside_map = pose(1000.0, 1000.0);
  const auto request = waypoint_route_request(pose_outside_map);

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  expect_fail_response_with_code(result, SetRoutePointsResponse::ERROR_PLANNER_FAILED);
  expect_states_transition({RouteState::ROUTING, RouteState::UNSET});
}

TEST_F(MissionPlannerTest, SetWaypointRoutePlansRouteToGoalLanelet)
{
  // Arrange
  const auto request = waypoint_route_request(pose(90.0));

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->header.frame_id, "map");
  EXPECT_EQ(result.route->segments.back().preferred_primitive.id, second_lanelet_id);
  EXPECT_DOUBLE_EQ(result.route->start_pose.position.x, start_x);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 90.0);
  EXPECT_TRUE(result.route_marker.has_value());
  EXPECT_TRUE(result.goal_footprint_marker.has_value());
  expect_states_transition({RouteState::ROUTING, RouteState::SET});
}

TEST_F(MissionPlannerTest, SetWaypointRouteTransformsWaypointsAndGoalIntoMapFrame)
{
  // Arrange
  tf_buffer.setTransform(transform_to_map(), "test", true);
  // In the sensor frame the waypoint and the goal are at x = 10 and x = 60, i.e. at x = 40 and
  // x = 90 in the map frame.
  const auto request = waypoint_route_request(pose(60.0), {pose(10.0)}, non_map_frame);

  auto mission_planner = create_initialized_mission_planner();

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->segments.back().preferred_primitive.id, second_lanelet_id);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 60.0 + map_frame_transform_x);
}

TEST_F(MissionPlannerTest, OnOdometryChangesStateToArrivedWhenStoppedAtGoal)
{
  // Arrange
  const auto goal_pose = pose(40.0);
  const auto request = lanelet_route_request({first_lanelet_id}, goal_pose);

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(request);
  states.clear();

  // Act
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);

  // Assert
  expect_states_transition({RouteState::ARRIVED});
}

TEST_F(MissionPlannerTest, SetLaneletRouteAfterArrivalFailsWithInvalidState)
{
  // Arrange
  const auto goal_pose = pose(40.0);
  const auto new_goal_pose = pose(90.0);
  const auto first_request = lanelet_route_request({first_lanelet_id}, goal_pose);
  const auto second_request = lanelet_route_request({}, new_goal_pose);

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(first_request);
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);

  // Act
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_fail_response_with_code(result, SetRouteResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, SetWaypointRouteAfterArrivalFailsWithInvalidState)
{
  // Arrange
  const auto goal_pose = pose(40.0);
  const auto new_goal_pose = pose(90.0);
  const auto first_request = waypoint_route_request(goal_pose);
  const auto second_request = waypoint_route_request(new_goal_pose);

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_waypoint_route(first_request);
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);

  // Act
  const auto result = mission_planner.set_waypoint_route(second_request);

  // Assert
  expect_fail_response_with_code(result, SetRoutePointsResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, ClearRouteAfterArrivalAllowsSettingANewRoute)
{
  // Arrange
  const auto goal_pose = pose(40.0);
  const auto new_goal_pose = pose(90.0);
  const auto first_request = lanelet_route_request({first_lanelet_id}, goal_pose);
  const auto second_request =
    lanelet_route_request({first_lanelet_id, second_lanelet_id}, new_goal_pose);

  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(first_request);
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);
  mission_planner.clear_route();

  // Act
  const auto result = mission_planner.set_lanelet_route(second_request);

  // Assert
  expect_success_response(result);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->segments.size(), 2U);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

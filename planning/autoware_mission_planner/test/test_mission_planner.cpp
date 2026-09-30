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
constexpr lanelet::Id FIRST_LANELET_ID = 1000;
constexpr lanelet::Id SECOND_LANELET_ID = 1001;

constexpr double map_frame_transform_x = 30.0;
constexpr char non_map_frame[] = "sensor_frame";

constexpr double reroute_time_threshold = 10.0;
constexpr double minimum_reroute_length = 30.0;
constexpr double arrival_check_duration = 1.0;

// x of the pose where the vehicle starts in every test (see make_start_odometry()).
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

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(make_road_lanelet(FIRST_LANELET_ID, left_0, left_50, right_0, right_50));
  lanelet_map->add(make_road_lanelet(SECOND_LANELET_ID, left_50, left_100, right_50, right_100));

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

// NOTE: values below mirror autoware_test_utils/config/test_vehicle_info.param.yaml
autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    /* wheel_radius_m= */ 0.383, /* wheel_width_m= */ 0.235, /* wheel_base_m= */ 2.79,
    /* wheel_tread_m= */ 1.64, /* front_overhang_m= */ 1.0, /* rear_overhang_m= */ 1.1,
    /* left_overhang_m= */ 0.128, /* right_overhang_m= */ 0.128, /* vehicle_height_m= */ 2.5,
    /* max_steer_angle_rad= */ 0.70);
}

// NOTE: values below mirror autoware_mission_planner/config/mission_planner.param.yaml
MissionPlannerConfig make_default_config()
{
  MissionPlannerConfig config;
  config.map_frame = "map";
  config.reroute_time_threshold = reroute_time_threshold;
  config.minimum_reroute_length = minimum_reroute_length;
  config.allow_reroute_in_autonomous_mode = false;
  config.arrival_checker_threshold.distance = 1.0;
  config.arrival_checker_threshold.angle = 45.0 * M_PI / 180.0;
  config.arrival_checker_threshold.duration = arrival_check_duration;
  config.default_planner_parameters.goal_angle_threshold_deg = 45.0;
  config.default_planner_parameters.enable_correct_goal_pose = false;
  config.default_planner_parameters.consider_no_drivable_lanes = false;
  config.default_planner_parameters.check_footprint_inside_lanes = true;
  config.vehicle_info = make_vehicle_info();
  return config;
}

Pose make_pose(const double x, const double y = 0.0)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

Odometry::ConstSharedPtr make_odometry(
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
Odometry::ConstSharedPtr make_start_odometry(const double velocity = 0.0)
{
  return make_odometry(make_pose(start_x), velocity);
}

OperationModeState::ConstSharedPtr make_operation_mode_state(
  const uint8_t mode, const bool is_autoware_control_enabled)
{
  OperationModeState state;
  state.mode = mode;
  state.is_autoware_control_enabled = is_autoware_control_enabled;
  return std::make_shared<OperationModeState>(state);
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

SetLaneletRoute::Request make_lanelet_route_request(
  const std::vector<lanelet::Id> & lanelet_ids, const Pose & goal_pose,
  const std::string & frame_id = "map")
{
  SetLaneletRoute::Request request;
  request.header.frame_id = frame_id;
  request.goal_pose = goal_pose;
  for (const auto lanelet_id : lanelet_ids) {
    request.segments.push_back(make_segment(lanelet_id));
  }
  return request;
}

SetWaypointRoute::Request make_waypoint_route_request(
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
TransformStamped make_transform_to_map()
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
    mission_planner.on_odometry(make_odometry(pose, 0.0, time_sec));
  }
}

// Returns a state change callback that appends every notified state to `states`. `states` must
// outlive the mission planner that holds the callback.
MissionPlanner::ChangeStateCallback record_states(std::vector<RouteState::_state_type> & states)
{
  return [&states](const auto state) { states.push_back(state); };
}

// Owns the objects that a mission planner refers to, so that the mission planners created by the
// fixture stay valid for the whole test. Every state change notified by these mission planners is
// appended to `states`.
class MissionPlannerTest : public ::testing::Test
{
protected:
  // Creates a mission planner that has received neither a map nor odometry.
  MissionPlanner create_mission_planner(const MissionPlannerConfig & config = make_default_config())
  {
    return MissionPlanner(config, tf_buffer, record_states(states));
  }

  // Creates a mission planner in the ready state (RouteState::UNSET) by feeding the map and the
  // odometry of the vehicle stopped at start_x. The states notified during the initialization are
  // discarded.
  MissionPlanner create_initialized_mission_planner(
    const MissionPlannerConfig & config = make_default_config())
  {
    auto mission_planner = create_mission_planner(config);
    mission_planner.on_map(std::make_shared<LaneletMapBin>(create_map()));
    mission_planner.on_odometry(make_start_odometry());
    mission_planner.check_initialization();
    states.clear();
    return mission_planner;
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
  mission_planner.on_odometry(make_start_odometry());

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
  mission_planner.on_odometry(make_start_odometry());

  // Act
  const auto is_initialized = mission_planner.check_initialization();

  // Assert
  EXPECT_TRUE(is_initialized);
  EXPECT_EQ(states, std::vector<RouteState::_state_type>{RouteState::UNSET});
}

TEST_F(MissionPlannerTest, ClearRouteBeforeInitializationHasNoEffect)
{
  // Arrange
  auto mission_planner = create_mission_planner();

  // Act
  const auto response = mission_planner.clear_route();

  // Assert
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
  EXPECT_EQ(states, std::vector<RouteState::_state_type>{RouteState::UNSET});
}

TEST_F(MissionPlannerTest, SetLaneletRouteBeforeInitializationFailsWithInvalidState)
{
  // Arrange
  auto mission_planner = create_mission_planner();
  const auto request = make_lanelet_route_request({}, make_pose(40.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_INVALID_STATE);
  EXPECT_FALSE(result.route.has_value());
}

TEST_F(MissionPlannerTest, SetWaypointRouteBeforeInitializationFailsWithInvalidState)
{
  // Arrange
  auto mission_planner = create_mission_planner();
  const auto request = make_waypoint_route_request(make_pose(90.0));

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRoutePointsResponse::ERROR_INVALID_STATE);
  EXPECT_FALSE(result.route.has_value());
}

TEST_F(MissionPlannerTest, SetLaneletRouteWithoutSegmentsFailsAndRestoresUnsetState)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_lanelet_route_request({}, make_pose(40.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_PLANNER_FAILED);
  EXPECT_FALSE(result.route.has_value());
  EXPECT_EQ(states, (std::vector<RouteState::_state_type>{RouteState::ROUTING, RouteState::UNSET}));
}

TEST_F(MissionPlannerTest, SetLaneletRouteFailsWhenTransformToMapIsUnavailable)
{
  // Arrange
  // The transform of the request frame is never registered in tf_buffer.
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_lanelet_route_request({}, make_pose(10.0), non_map_frame);

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, ResponseStatus::TRANSFORM_ERROR);
  EXPECT_FALSE(result.route.has_value());
}

TEST_F(MissionPlannerTest, SetWaypointRouteFailsWhenTransformToMapIsUnavailable)
{
  // Arrange
  // The transform of the request frame is never registered in tf_buffer.
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_waypoint_route_request(make_pose(60.0), {}, non_map_frame);

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, ResponseStatus::TRANSFORM_ERROR);
  EXPECT_FALSE(result.route.has_value());
}

TEST_F(MissionPlannerTest, SetLaneletRouteSucceedsAfterInitialization)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  ASSERT_EQ(result.route->segments.size(), 1U);
  EXPECT_EQ(result.route->segments.front().preferred_primitive.id, FIRST_LANELET_ID);
  EXPECT_EQ(result.route->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.route->start_pose.position.x, start_x);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 40.0);
  EXPECT_DOUBLE_EQ(result.initial_pose.position.x, start_x);
  EXPECT_TRUE(result.route_marker.has_value());
  EXPECT_EQ(states, (std::vector<RouteState::_state_type>{RouteState::ROUTING, RouteState::SET}));
}

TEST_F(MissionPlannerTest, SetLaneletRouteTransformsGoalPoseIntoMapFrame)
{
  // Arrange
  tf_buffer.setTransform(make_transform_to_map(), "test", true);
  auto mission_planner = create_initialized_mission_planner();
  const auto request =
    make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(10.0), non_map_frame);

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 10.0 + map_frame_transform_x);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteFailsWhenOperationModeStateIsNotReceived)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(
    make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0)));
  const auto request = make_lanelet_route_request({}, make_pose(90.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_PLANNER_UNREADY);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteFailsWhenNotAllowedInAutonomousMode)
{
  // Arrange
  auto config = make_default_config();
  config.allow_reroute_in_autonomous_mode = false;
  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, true));
  mission_planner.set_lanelet_route(
    make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0)));
  const auto request = make_lanelet_route_request({}, make_pose(90.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteSucceedsWhenNotInAutonomousMode)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const double driving_velocity = 5.0;
  mission_planner.on_odometry(make_start_odometry(driving_velocity));
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::STOP, false));
  mission_planner.set_lanelet_route(
    make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0)));
  const auto request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(20.0));

  // Act
  // The reroute safety check is skipped outside autonomous mode, so even a short new route is
  // accepted while driving.
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetLaneletRouteRerouteSucceedsWhenAutowareControlIsDisabled)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  // The vehicle must be moving: while stopped, the reroute safety check always passes, so this test
  // could not tell whether the check is skipped.
  const double driving_velocity = 5.0;
  mission_planner.on_odometry(make_start_odometry(driving_velocity));
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, false));
  mission_planner.set_lanelet_route(
    make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0)));
  const auto request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(20.0));

  // Act
  // The vehicle is not driven by Autoware, so it is not treated as autonomous driving and the
  // reroute safety check is skipped.
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 20.0);
}

TEST_F(MissionPlannerTest, SetLaneletRouteTwiceWhileStoppedInAutonomousModeReroutesToSecondRoute)
{
  // Arrange
  auto config = make_default_config();
  config.allow_reroute_in_autonomous_mode = true;
  auto mission_planner = create_initialized_mission_planner(config);
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, true));
  const auto first_request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0));
  const auto second_request =
    make_lanelet_route_request({FIRST_LANELET_ID, SECOND_LANELET_ID}, make_pose(90.0));

  // Act
  // The vehicle is stopped, so the reroute safety check passes regardless of the route length.
  const auto first_result = mission_planner.set_lanelet_route(first_request);
  const auto second_result = mission_planner.set_lanelet_route(second_request);

  // Assert
  ASSERT_TRUE(first_result.response.status.success);
  EXPECT_TRUE(second_result.response.status.success);
  ASSERT_TRUE(second_result.route.has_value());
  EXPECT_EQ(second_result.route->segments.size(), 2U);
  EXPECT_EQ(
    states, (std::vector<RouteState::_state_type>{
              RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET}));
}

TEST_F(MissionPlannerTest, SetLaneletRouteTwiceWhileDrivingKeepsFirstRouteWhenRerouteIsUnsafe)
{
  // Arrange
  auto config = make_default_config();
  config.allow_reroute_in_autonomous_mode = true;
  auto mission_planner = create_initialized_mission_planner(config);
  // Driving fast enough that the required safety length (velocity * reroute_time_threshold = 100 m)
  // exceeds the 30 m shared with the new route.
  const double high_velocity = 10.0;
  mission_planner.on_odometry(make_start_odometry(high_velocity));
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, true));
  const auto first_request =
    make_lanelet_route_request({FIRST_LANELET_ID, SECOND_LANELET_ID}, make_pose(90.0));
  const auto second_request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0));

  // Act
  const auto first_result = mission_planner.set_lanelet_route(first_request);
  const auto second_result = mission_planner.set_lanelet_route(second_request);

  // Assert
  ASSERT_TRUE(first_result.response.status.success);
  EXPECT_FALSE(second_result.response.status.success);
  EXPECT_EQ(second_result.response.status.code, SetRouteResponse::ERROR_REROUTE_FAILED);
  EXPECT_TRUE(second_result.error_message.has_value());
  EXPECT_FALSE(second_result.route.has_value());
  EXPECT_EQ(
    states, (std::vector<RouteState::_state_type>{
              RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET}));
}

TEST_F(
  MissionPlannerTest, SetLaneletRouteRerouteFailsWhenSharedRouteIsShorterThanMinimumRerouteLength)
{
  // Arrange
  auto config = make_default_config();
  config.allow_reroute_in_autonomous_mode = true;
  config.minimum_reroute_length = 40.0;
  auto mission_planner = create_initialized_mission_planner(config);
  // Driving slowly, so the velocity-dependent safety length (1 m/s * 10 s = 10 m) is shorter than
  // the 30 m shared with the new route and only minimum_reroute_length can reject the reroute.
  const double low_velocity = 1.0;
  mission_planner.on_odometry(make_start_odometry(low_velocity));
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, true));
  mission_planner.set_lanelet_route(
    make_lanelet_route_request({FIRST_LANELET_ID, SECOND_LANELET_ID}, make_pose(90.0)));
  const auto request = make_lanelet_route_request({FIRST_LANELET_ID}, make_pose(40.0));

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_REROUTE_FAILED);
  EXPECT_TRUE(result.error_message.has_value());
}

TEST_F(MissionPlannerTest, SetWaypointRouteRerouteFailsWhenOperationModeStateIsNotReceived)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_waypoint_route(make_waypoint_route_request(make_pose(90.0)));
  const auto request = make_waypoint_route_request(make_pose(40.0));

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRoutePointsResponse::ERROR_PLANNER_UNREADY);
}

TEST_F(MissionPlannerTest, SetWaypointRouteTwiceWhileDrivingKeepsFirstRouteWhenRerouteIsUnsafe)
{
  // Arrange
  auto config = make_default_config();
  config.allow_reroute_in_autonomous_mode = true;
  auto mission_planner = create_initialized_mission_planner(config);
  // Driving fast enough that the required safety length (velocity * reroute_time_threshold = 100 m)
  // exceeds the 30 m shared with the new route.
  const double high_velocity = 10.0;
  mission_planner.on_odometry(make_start_odometry(high_velocity));
  mission_planner.on_operation_mode_state(
    make_operation_mode_state(OperationModeState::AUTONOMOUS, true));
  const auto first_request = make_waypoint_route_request(make_pose(90.0));
  const auto second_request = make_waypoint_route_request(make_pose(40.0));

  // Act
  const auto first_result = mission_planner.set_waypoint_route(first_request);
  const auto second_result = mission_planner.set_waypoint_route(second_request);

  // Assert
  ASSERT_TRUE(first_result.response.status.success);
  EXPECT_FALSE(second_result.response.status.success);
  EXPECT_EQ(second_result.response.status.code, SetRoutePointsResponse::ERROR_REROUTE_FAILED);
  EXPECT_TRUE(second_result.error_message.has_value());
  EXPECT_FALSE(second_result.route.has_value());
  EXPECT_EQ(
    states, (std::vector<RouteState::_state_type>{
              RouteState::ROUTING, RouteState::SET, RouteState::REROUTING, RouteState::SET}));
}

TEST_F(MissionPlannerTest, SetWaypointRouteFailsWhenGoalIsOutsideTheMap)
{
  // Arrange
  const auto pose_outside_map = make_pose(1000.0, 1000.0);
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_waypoint_route_request(pose_outside_map);

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRoutePointsResponse::ERROR_PLANNER_FAILED);
  EXPECT_FALSE(result.route.has_value());
  EXPECT_EQ(states, (std::vector<RouteState::_state_type>{RouteState::ROUTING, RouteState::UNSET}));
}

TEST_F(MissionPlannerTest, SetWaypointRoutePlansRouteToGoalLanelet)
{
  // Arrange
  auto mission_planner = create_initialized_mission_planner();
  const auto request = make_waypoint_route_request(make_pose(90.0));

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->header.frame_id, "map");
  EXPECT_EQ(result.route->segments.back().preferred_primitive.id, SECOND_LANELET_ID);
  EXPECT_DOUBLE_EQ(result.route->start_pose.position.x, start_x);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 90.0);
  EXPECT_TRUE(result.route_marker.has_value());
  EXPECT_TRUE(result.goal_footprint_marker.has_value());
  EXPECT_EQ(states, (std::vector<RouteState::_state_type>{RouteState::ROUTING, RouteState::SET}));
}

TEST_F(MissionPlannerTest, SetWaypointRouteTransformsWaypointsAndGoalIntoMapFrame)
{
  // Arrange
  tf_buffer.setTransform(make_transform_to_map(), "test", true);
  auto mission_planner = create_initialized_mission_planner();
  // In the sensor frame the waypoint and the goal are at x = 10 and x = 60, i.e. at x = 40 and
  // x = 90 in the map frame.
  const auto request =
    make_waypoint_route_request(make_pose(60.0), {make_pose(10.0)}, non_map_frame);

  // Act
  const auto result = mission_planner.set_waypoint_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->segments.back().preferred_primitive.id, SECOND_LANELET_ID);
  EXPECT_DOUBLE_EQ(result.route->goal_pose.position.x, 60.0 + map_frame_transform_x);
}

TEST_F(MissionPlannerTest, OnOdometryChangesStateToArrivedWhenStoppedAtGoal)
{
  // Arrange
  const auto goal_pose = make_pose(40.0);
  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(make_lanelet_route_request({FIRST_LANELET_ID}, goal_pose));
  states.clear();

  // Act
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);

  // Assert
  EXPECT_EQ(states, std::vector<RouteState::_state_type>{RouteState::ARRIVED});
}

TEST_F(MissionPlannerTest, SetLaneletRouteAfterArrivalFailsWithInvalidState)
{
  // Arrange
  const auto goal_pose = make_pose(40.0);
  const auto new_goal_pose = make_pose(90.0);
  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(make_lanelet_route_request({FIRST_LANELET_ID}, goal_pose));
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);
  const auto request = make_lanelet_route_request({}, new_goal_pose);

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_FALSE(result.response.status.success);
  EXPECT_EQ(result.response.status.code, SetRouteResponse::ERROR_INVALID_STATE);
}

TEST_F(MissionPlannerTest, ClearRouteAfterArrivalAllowsSettingANewRoute)
{
  // Arrange
  const auto goal_pose = make_pose(40.0);
  const auto new_goal_pose = make_pose(90.0);
  auto mission_planner = create_initialized_mission_planner();
  mission_planner.set_lanelet_route(make_lanelet_route_request({FIRST_LANELET_ID}, goal_pose));
  stay_stopped_at(mission_planner, goal_pose, arrival_check_duration + 0.5);
  mission_planner.clear_route();
  const auto request =
    make_lanelet_route_request({FIRST_LANELET_ID, SECOND_LANELET_ID}, new_goal_pose);

  // Act
  const auto result = mission_planner.set_lanelet_route(request);

  // Assert
  EXPECT_TRUE(result.response.status.success);
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.route->segments.size(), 2U);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

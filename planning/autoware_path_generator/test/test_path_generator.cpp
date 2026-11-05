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

#include "autoware/path_generator/path_generator.hpp"
#include "utils_test.hpp"

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-1F;

PathGeneratorConfig create_default_config()
{
  PathGeneratorConfig config;
  config.vehicle_length_m = 4.79;
  config.max_longitudinal_offset_m = 3.79;
  return config;
}

::path_generator::Params create_default_params()
{
  ::path_generator::Params params;
  params.planning_hz = 10.0;
  params.path_length.backward = 5.0;
  params.path_length.forward = 50.0;
  params.waypoint.connection_gradient_from_centerline = 10.0;
  params.turn_signal.search_time = 3.0;
  params.turn_signal.search_distance = 30.0;
  params.turn_signal.resampling_interval = 1.0;
  params.turn_signal.angle_threshold_deg = 15.0;
  params.goal_connection.connection_section_length = 7.5;
  params.goal_connection.pre_goal_offset = 1.0;
  params.ego_nearest_dist_threshold = 3.0;
  params.ego_nearest_yaw_threshold = 1.047;  // ~60 deg
  return params;
}

nav_msgs::msg::Odometry create_odom(const double x, const double y, const double linear_x = 5.0)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp.sec = 123;
  odom.header.stamp.nanosec = 456000000;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = linear_x;
  return odom;
}
}  // namespace

// Expects constructor to throw exception when vehicle_length_m is negative.
TEST(PathGeneratorTest, ThrowsExceptionWhenVehicleLengthIsNegative)
{
  // Arrange
  auto config = create_default_config();
  config.vehicle_length_m = -1.0;

  // Act & Assert
  EXPECT_THROW(PathGeneratorCore{config}, std::invalid_argument);
}

// Expects constructor to throw exception when max_longitudinal_offset_m is negative.
TEST(PathGeneratorTest, ThrowsExceptionWhenMaxLongitudinalOffsetIsNegative)
{
  // Arrange
  auto config = create_default_config();
  config.max_longitudinal_offset_m = -0.5;

  // Act & Assert
  EXPECT_THROW(PathGeneratorCore{config}, std::invalid_argument);
}

// Expects generate to return error when map or route pointer is missing.
TEST(PathGeneratorTest, ReturnsErrorWhenMapOrRouteMissing)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.odometry = create_odom(10.5, 0.0);
  input.params = create_default_params();

  // Act (Null map)
  auto result_no_map = generator.generate(input);

  // Assert
  EXPECT_FALSE(result_no_map.has_value());
  EXPECT_EQ(result_no_map.error(), "Vector map is not provided");

  // Act (Null route)
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());
  auto result_no_route = generator.generate(input);

  // Assert
  EXPECT_FALSE(result_no_route.has_value());
  EXPECT_EQ(result_no_route.error(), "Route is not provided");
}

// Expects generate to return error when input route has no segments.
TEST(PathGeneratorTest, ReturnsErrorWhenRouteIsEmpty)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());
  auto empty_route = create_mock_route();
  empty_route.segments.clear();
  input.route_ptr = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(empty_route);
  input.odometry = create_odom(10.5, 0.0);
  input.params = create_default_params();

  // Act
  const auto result = generator.generate(input);

  // Assert
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "Input route is empty");
}

// Expects generate to return error when route primitives are empty and RouteManager creation fails.
TEST(PathGeneratorTest, ReturnsErrorWhenRoutePrimitivesEmptyAndRouteManagerCannotBeCreated)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());

  auto route = create_mock_route();
  for (auto & segment : route.segments) {
    segment.primitives.clear();
  }
  input.route_ptr = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(route);
  input.odometry = create_odom(10.5, 0.0);
  input.params = create_default_params();

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "Failed to create route manager");
}

// Expects generate to return error when initial ego pose is far away so trajectory build fails.
TEST(PathGeneratorTest, ReturnsErrorWhenTrajectoryBuildFails)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());
  input.route_ptr =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_route());
  // Initial pose is 1000m away from the lanelet
  input.odometry = create_odom(1000.0, 1000.0);
  input.params = create_default_params();

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "Failed to build trajectory from path points");
}

// Expects generate to successfully output path and signal commands on nominal input.
TEST(PathGeneratorTest, GeneratesPathSuccessfullyWhenValidInputProvided)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());
  input.route_ptr =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_route());
  input.odometry = create_odom(10.5, 0.0);
  input.params = create_default_params();

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->path.points.empty());
  EXPECT_EQ(result->path.header.frame_id, "map");
  EXPECT_EQ(result->path.header.stamp, input.odometry.header.stamp);
  EXPECT_FALSE(result->path.left_bound.empty());
  EXPECT_FALSE(result->path.right_bound.empty());
  EXPECT_EQ(result->turn_indicators_cmd.command, TurnIndicatorsCommand::NO_COMMAND);
  EXPECT_EQ(result->hazard_lights_cmd.command, HazardLightsCommand::NO_COMMAND);
}

// Expects RouteManager to be reused across consecutive calls with same pointers, and rebuilt when
// pointers change.
TEST(PathGeneratorTest, CachesRouteManagerAndRebuildsWhenPointersChange)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  const auto map_ptr_1 =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_common_map_bin());
  const auto route_ptr_1 =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_route());

  input.map_ptr = map_ptr_1;
  input.route_ptr = route_ptr_1;
  input.odometry = create_odom(10.5, 0.0);
  input.params = create_default_params();

  // Act 1: Initial call (builds RouteManager)
  const auto result1 = generator.generate(input);
  ASSERT_TRUE(result1.has_value()) << result1.error();

  // Act 2: Advance ego pose slightly with identical map/route pointers (reuses cached RouteManager)
  input.odometry = create_odom(15.0, 0.0);
  const auto result2 = generator.generate(input);
  ASSERT_TRUE(result2.has_value()) << result2.error();
  EXPECT_FALSE(result2->path.points.empty());

  // Act 3: Update route pointer to a new route instance (triggers RouteManager rebuild)
  const auto route_ptr_2 =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_route());
  input.route_ptr = route_ptr_2;
  const auto result3 = generator.generate(input);
  ASSERT_TRUE(result3.has_value()) << result3.error();
  EXPECT_FALSE(result3->path.points.empty());
}

// Expects turn indicators to be commanded when approaching a lanelet with turn_direction.
TEST(PathGeneratorTest, GeneratesTurnSignalWhenApproachingTurn)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr = std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(
    create_mock_three_lanelets_map_bin("left"));
  input.route_ptr =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_three_lanelets_route());

  // Ego at x = 10.0 on lane 1001 (0 to 20m), moving at 5 m/s approaching left-turn lane 1002 (20 to
  // 40m)
  input.odometry = create_odom(10.0, 0.0, 5.0);
  input.params = create_default_params();

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->turn_indicators_cmd.command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(result->turn_indicators_cmd.stamp, input.odometry.header.stamp);
  EXPECT_EQ(result->hazard_lights_cmd.command, HazardLightsCommand::NO_COMMAND);
}

// Expects path to be truncated before self-intersection on a loop route.
TEST(PathGeneratorTest, TruncatesPathWhenLoopSelfIntersectionDetected)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_loop_map_bin());
  input.route_ptr =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(create_mock_loop_route());
  input.odometry = create_odom(5.5, 0.0);

  auto params = create_default_params();
  params.path_length.forward = 200.0;  // Long enough to loop around
  input.params = params;

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->path.points.empty());

  // Check truncation: Path should not cross into second pass over X=20 on L1
  for (const auto & pt : result->path.points) {
    if (pt.point.pose.position.x > 38.0) {
      // Must be on the northern loop or right turn, not looping back into L1
      EXPECT_GE(pt.point.pose.position.y, -2.5);
    }
  }
}

// Expects path to connect smoothly to goal when goal is within forward planning distance.
TEST(PathGeneratorTest, ConnectsPathToGoalWhenGoalWithinForwardDistance)
{
  // Arrange
  PathGeneratorCore generator{create_default_config()};
  PathGeneratorInput input;
  input.map_ptr =
    std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(create_mock_dense_map_bin());

  auto route = create_mock_route();
  route.goal_pose.position.x = 45.0;
  route.goal_pose.position.y = 1.0;  // Slight lateral offset inside lane
  input.route_ptr = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(route);

  input.odometry = create_odom(10.5, 0.0);
  auto params = create_default_params();
  params.path_length.forward = 50.0;  // Goal at 45.0 is within 10.5 + 50.0 = 60.5
  input.params = params;

  // Act
  const auto result = generator.generate(input);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_FALSE(result->path.points.empty());

  const auto & final_point = result->path.points.back().point.pose.position;
  EXPECT_NEAR(final_point.x, route.goal_pose.position.x, near_tol);
  EXPECT_NEAR(final_point.y, route.goal_pose.position.y, near_tol);
}

}  // namespace autoware::path_generator

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

}  // namespace autoware::path_generator

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

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/constants.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{

class MotionVelocityPlannerCommonUtilsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MotionVelocityPlannerCommonUtilsTest, GetTargetObjectTypeSupportsAnimalAndHazard)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("target.unknown", false);
  options.append_parameter_override("target.car", false);
  options.append_parameter_override("target.truck", false);
  options.append_parameter_override("target.bus", false);
  options.append_parameter_override("target.trailer", false);
  options.append_parameter_override("target.motorcycle", false);
  options.append_parameter_override("target.bicycle", false);
  options.append_parameter_override("target.pedestrian", false);
  options.append_parameter_override("target.animal", true);
  options.append_parameter_override("target.hazard", true);

  auto node = std::make_shared<rclcpp::Node>("test_get_target_object_type", options);
  const auto types = get_target_object_type(*node, "target.");

  EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::ANIMAL), types.end());
  EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::HAZARD), types.end());
  EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::UNKNOWN), types.end());
}

TEST_F(MotionVelocityPlannerCommonUtilsTest, GetTargetObjectTypeDefaultsMissingLabelsToFalse)
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("target.unknown", true);
  options.append_parameter_override("target.car", false);
  options.append_parameter_override("target.truck", false);
  options.append_parameter_override("target.bus", false);
  options.append_parameter_override("target.trailer", false);
  options.append_parameter_override("target.motorcycle", false);
  options.append_parameter_override("target.bicycle", false);
  options.append_parameter_override("target.pedestrian", false);

  auto node = std::make_shared<rclcpp::Node>("test_get_target_object_type_defaults", options);

  EXPECT_NO_THROW({
    const auto types = get_target_object_type(*node, "target.");
    EXPECT_NE(std::find(types.begin(), types.end(), ObjectClassification::UNKNOWN), types.end());
    EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::ANIMAL), types.end());
    EXPECT_EQ(std::find(types.begin(), types.end(), ObjectClassification::HAZARD), types.end());
  });
}

}  // namespace autoware::motion_velocity_planner::utils

namespace
{
using autoware::motion_velocity_planner::utils::calc_distance_to_front_object;
using autoware::motion_velocity_planner::utils::calc_object_possible_max_dist_from_center;
using autoware::motion_velocity_planner::utils::concat_vectors;
using autoware::motion_velocity_planner::utils::get_extended_trajectory_points;
using autoware::motion_velocity_planner::utils::get_index_with_longitudinal_offset;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;

// Build a straight trajectory along +x with identity orientation and a forward longitudinal
// velocity so that direction detection returns "driving forward".
std::vector<TrajectoryPoint> make_straight_forward_trajectory(
  const size_t num_points, const double step)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i) * step;
    p.pose.position.y = 0.0;
    p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    p.longitudinal_velocity_mps = 1.0;
    points.push_back(p);
  }
  return points;
}

// Build a trajectory following a circular arc of the given radius (left turn, centre at (0, R))
// with each point's orientation set to the arc tangent, so direction detection returns "driving
// forward" just like make_straight_forward_trajectory.
std::vector<TrajectoryPoint> make_arc_forward_trajectory(
  const size_t num_points, const double step, const double radius)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = static_cast<double>(i) * step / radius;
    TrajectoryPoint p;
    p.pose.position.x = radius * std::sin(theta);
    p.pose.position.y = radius * (1.0 - std::cos(theta));
    p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(theta);
    p.longitudinal_velocity_mps = 1.0;
    points.push_back(p);
  }
  return points;
}

// Shortest distance from a point to the polyline through the trajectory points from begin_index
// onwards. The collision check sweeps a footprint between consecutive points, so distance to the
// segments is what decides a hit, not distance to the nearest sampled point.
double distance_to_polyline(
  const std::vector<TrajectoryPoint> & points, const size_t begin_index,
  const geometry_msgs::msg::Point & query)
{
  double closest = std::numeric_limits<double>::max();
  for (size_t i = begin_index; i + 1 < points.size(); ++i) {
    const auto & a = points.at(i).pose.position;
    const auto & b = points.at(i + 1).pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t =
      length_squared > 0.0
        ? std::clamp(((query.x - a.x) * dx + (query.y - a.y) * dy) / length_squared, 0.0, 1.0)
        : 0.0;
    closest = std::min(closest, std::hypot(query.x - (a.x + t * dx), query.y - (a.y + t * dy)));
  }
  return closest;
}

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}
}  // namespace

// ----------------------------- calc_object_possible_max_dist_from_center -----------------------

TEST(MvpUtilsMaxDist, BoundingBoxReturnsHalfDiagonal)
{
  Shape shape;
  shape.type = Shape::BOUNDING_BOX;
  shape.dimensions.x = 4.0;
  shape.dimensions.y = 3.0;
  // half-diagonal = hypot(2, 1.5) = 2.5
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 2.5, 1e-9);
}

TEST(MvpUtilsMaxDist, CylinderReturnsRadius)
{
  Shape shape;
  shape.type = Shape::CYLINDER;
  shape.dimensions.x = 6.0;  // diameter
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 3.0, 1e-9);
}

TEST(MvpUtilsMaxDist, PolygonReturnsFarthestPointDistance)
{
  Shape shape;
  shape.type = Shape::POLYGON;
  const std::vector<std::pair<double, double>> corners{{1.0, 0.0}, {0.0, 2.0}, {-3.0, -4.0}};
  for (const auto & [x, y] : corners) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    shape.footprint.points.push_back(p);
  }
  // farthest point is (-3, -4): hypot = 5
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 5.0, 1e-6);
}

TEST(MvpUtilsMaxDist, EmptyPolygonReturnsZero)
{
  Shape shape;
  shape.type = Shape::POLYGON;
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 0.0, 1e-9);
}

TEST(MvpUtilsMaxDist, UnsupportedShapeThrowsLogicError)
{
  Shape shape;
  shape.type = 255;  // not a supported shape type
  EXPECT_THROW(calc_object_possible_max_dist_from_center(shape), std::logic_error);
}

// ----------------------------- get_index_with_longitudinal_offset ------------------------------

TEST(MvpUtilsLongitudinalOffset, EmptyPointsThrows)
{
  const std::vector<TrajectoryPoint> empty_points;
  EXPECT_THROW(
    get_index_with_longitudinal_offset(empty_points, 1.0, std::nullopt), std::logic_error);
}

TEST(MvpUtilsLongitudinalOffset, StartIndexOutOfRangeThrows)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);
  EXPECT_THROW(
    get_index_with_longitudinal_offset(points, 1.0, std::optional<size_t>(3)), std::out_of_range);
}

TEST(MvpUtilsLongitudinalOffset, ForwardRoundsToNearerEndpoint)
{
  // points at x = 0, 1, 2, 3, 4 (1 m spacing). For a forward offset the function finds the segment
  // [i, i+1] whose cumulative length first reaches the offset, then returns the endpoint of that
  // segment that is closer to the offset position.
  const auto points = make_straight_forward_trajectory(5, 1.0);

  // offset 2.4: reached on segment [2, 3] (cumulative sum = 3.0 at i = 2). distance from the offset
  // to point 2 (front_length) = 0.4, to point 3 (back_length) = 0.6 -> point 2 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.4, std::nullopt), 2u);

  // offset 2.1: front_length = 0.1, back_length = 0.9 -> point 2 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.1, std::nullopt), 2u);

  // offset 2.6: front_length = 0.6, back_length = 0.4 -> point 3 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.6, std::nullopt), 3u);
}

TEST(MvpUtilsLongitudinalOffset, ForwardOffsetBeyondEndReturnsLastIndex)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // total length 4.0
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 100.0, std::nullopt), 4u);
}

TEST(MvpUtilsLongitudinalOffset, BackwardFromDefaultStart)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);

  // Negative offset and no start_idx -> start from the last index (4) and walk backward.
  // offset -1.4: accumulate backward from index 4: i=4 covers segment [3, 4] (sum=1.0 < 1.4), i=3
  // covers segment [2, 3] (sum=2.0 >= 1.4), so the threshold is reached on segment [2, 3].
  // back_length = sum + offset = 2.0 - 1.4 = 0.6 (distance from offset to point 2 = points[i-1]),
  // front_length = seg - back = 1.0 - 0.6 = 0.4 (distance from offset to point 3 = points[i]).
  // front_length < back_length -> the offset is closer to point 3, so index i = 3 is returned.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, -1.4, std::nullopt), 3u);
}

TEST(MvpUtilsLongitudinalOffset, BackwardOffsetBeyondStartReturnsZero)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // total length 4.0
  EXPECT_EQ(get_index_with_longitudinal_offset(points, -100.0, std::nullopt), 0u);
}

// ----------------------------- get_extended_trajectory_points ----------------------------------

TEST(MvpUtilsExtend, ShortExtendDistanceReturnsInputUnchanged)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);
  // extend_distance below the internal min_step_length (0.1) -> input returned as-is.
  const auto result = get_extended_trajectory_points(points, 0.05, 2.0);
  ASSERT_EQ(result.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i].pose.position.x, points[i].pose.position.x);
  }
}

TEST(MvpUtilsExtend, AppendsIntermediateAndFinalPoints)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  const double extend_distance = 5.0;
  const double step_length = 2.0;

  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);

  // The loop steps by step_length up to extend_distance (2.0 and 4.0), then the final point lands
  // on extend_distance exactly.
  ASSERT_EQ(result.size(), points.size() + 3);
  EXPECT_NEAR(result[points.size()].pose.position.x, 2.0 + 2.0, 1e-6);      // x = 4.0
  EXPECT_NEAR(result[points.size() + 1].pose.position.x, 2.0 + 4.0, 1e-6);  // x = 6.0
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + extend_distance, 1e-6);  // x = 7.0
  // velocity is carried over from the goal point.
  EXPECT_DOUBLE_EQ(
    result.back().longitudinal_velocity_mps, points.back().longitudinal_velocity_mps);
}

TEST(MvpUtilsExtend, OnlyFinalPointWhenStepLargerThanDistance)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  const double extend_distance = 1.0;
  const double step_length = 2.0;

  // The loop condition (step_length < extend_distance - step_length -> 2 < -1) is false, so only
  // the single final point at extend_distance is appended.
  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);
  ASSERT_EQ(result.size(), points.size() + 1);
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + extend_distance, 1e-6);  // x = 3.0
}

TEST(MvpUtilsExtend, EmptyInputReturnsEmptyWithoutDereferencingBack)
{
  // Degenerate case: an empty trajectory has no goal point to extend from. With an
  // extend_distance >= min_step_length (0.1) the short-distance early return is skipped, so the
  // empty guard must prevent the back() dereference and return the input unchanged.
  const std::vector<TrajectoryPoint> empty_points;
  const auto result = get_extended_trajectory_points(empty_points, 5.0, 2.0);
  EXPECT_TRUE(result.empty());
}

// The goal-side extension follows the curvature of the road at the goal instead of the tangent,
// so on a circular arc the extended points stay on the arc. Anything more than a few millimetres
// of drift means the extension has gone back to a straight line.
TEST(MvpUtilsExtend, ExtensionFollowsLaneCurvature)
{
  constexpr double extend_distance = 6.0;
  constexpr double step_length = 2.0;
  for (const double radius : {5.0, 10.0, 20.0, 50.0}) {
    const auto points = make_arc_forward_trajectory(6, 1.0, radius);
    const auto result = get_extended_trajectory_points(points, extend_distance, step_length);
    ASSERT_GT(result.size(), points.size());
    for (size_t i = points.size(); i < result.size(); ++i) {
      const auto & p = result[i].pose.position;
      // Distance off the arc = how far the point sits from the arc's circle.
      const double drift = std::abs(std::hypot(p.x, p.y - radius) - radius);
      EXPECT_LT(drift, 1e-3) << "radius = " << radius << " m, extended point " << i;
      std::cout << "[drift] radius = " << radius << " m, point " << i << " -> drift = " << drift
                << " m" << std::endl;
    }
  }
}

// A straight trajectory must keep extending straight: the curvature estimate is zero there and the
// arc formula has to degrade to the tangent case rather than dividing by it.
TEST(MvpUtilsExtend, StraightTrajectoryStillExtendsStraight)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // last point at x = 4.0
  const auto result = get_extended_trajectory_points(points, 6.0, 2.0);
  ASSERT_GT(result.size(), points.size());
  for (size_t i = points.size(); i < result.size(); ++i) {
    EXPECT_NEAR(result[i].pose.position.y, 0.0, 1e-9) << "extended point " << i;
  }
}

// The extension samples every step_length so that the one-step collision polygons built from it
// stay tight against the road. A gap wider than step_length would cut the corner on a curve.
TEST(MvpUtilsExtend, ExtensionSpacesPointsByStepLength)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  constexpr double extend_distance = 6.0;
  constexpr double step_length = 2.0;
  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);
  ASSERT_GT(result.size(), points.size());
  double previous_x = points.back().pose.position.x;
  for (size_t i = points.size(); i < result.size(); ++i) {
    const double gap = result[i].pose.position.x - previous_x;
    EXPECT_GT(gap, 0.0) << "extended point " << i;
    EXPECT_LE(gap, step_length + 1e-6) << "extended point " << i;
    previous_x = result[i].pose.position.x;
  }
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + extend_distance, 1e-6);
}

// An extend_distance that is an exact multiple of step_length is the worst case for the sampling
// loop: it used to skip the loop entirely and emit a single point at the far end.
TEST(MvpUtilsExtend, ExactMultipleOfStepStillSamplesEveryStep)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  const auto result = get_extended_trajectory_points(points, 4.0, 2.0);

  ASSERT_EQ(result.size(), points.size() + 2);
  EXPECT_NEAR(result[points.size()].pose.position.x, 2.0 + 2.0, 1e-6);
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + 4.0, 1e-6);
}

// The extension is anchored on the untrimmed trajectory's terminal orientation, not on the
// orientation resampling gives the last decimated point. Feed a trajectory whose terminal pose
// deliberately disagrees with the spline fit and check the extension follows the terminal pose.
TEST(MvpUtilsDecimate, ExtensionUsesTheUntrimmedTerminalOrientation)
{
  constexpr double radius = 5.0;
  constexpr double step_length = 2.0;
  auto points = make_arc_forward_trajectory(25, 0.25, radius);

  // Rotate the terminal pose a long way off the arc tangent. A spline through the neighbouring
  // points cannot produce this, so the extension direction shows which orientation was used.
  const double true_tangent = 6.0 / radius;
  const double skewed = true_tangent + 0.5;
  points.back().pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(skewed);

  const auto result = autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    points, points.front().pose, 3.0, 1.046, step_length, 6.0);
  ASSERT_GT(result.size(), 2u);

  // The first extended point must sit step_length away along the skewed heading, not the tangent.
  const auto & goal = points.back().pose.position;
  const auto & first_extended = result.at(result.size() - 3).pose.position;
  const double heading_to_first = std::atan2(first_extended.y - goal.y, first_extended.x - goal.x);
  EXPECT_NEAR(heading_to_first, skewed + step_length / radius / 2.0, 0.05);
}

// Check that a point obstacle on a circular arc beyond the goal lies within the extended
// footprint. The lateral reach combines the vehicle half width and a nominal margin;
// a point obstacle does not contribute an additional object half width.
TEST(MvpUtilsExtend, ScenarioCurveKeepsPedestrianInsideExtendedFootprint)
{
  constexpr double lane_radius = 5.0;
  constexpr double extend_distance = 6.0;
  constexpr double step_length = 2.0;
  // Place the obstacle using a longitudinal offset, half vehicle length, gap, and half object
  // length. These fixed test dimensions put it between the final two extended points.
  constexpr double s_to_pedestrian = 1.0485 + 3.117 / 2.0 + 1.5 + 0.8;
  // Vehicle half width is half the wheel tread plus the lateral overhang.
  constexpr double ego_half_width = 0.975 / 2.0 + 0.1955;
  constexpr double nominal_lateral_margin = 0.1;  // obstacle_filtering.lateral_margin.nominal
  constexpr double pointcloud_reach = ego_half_width + nominal_lateral_margin;
  const auto points = make_arc_forward_trajectory(6, 1.0, lane_radius);
  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);
  const double theta_goal = 5.0 / lane_radius;  // last input point, 5 steps of 1 m
  const double theta_pedestrian = theta_goal + s_to_pedestrian / lane_radius;
  const auto pedestrian = make_point(
    lane_radius * std::sin(theta_pedestrian), lane_radius * (1.0 - std::cos(theta_pedestrian)));
  double reached_arc_length = 0.0;
  for (size_t i = points.size(); i < result.size(); ++i) {
    const auto & p = result.at(i).pose.position;
    const double theta = std::atan2(p.x, lane_radius - p.y);
    reached_arc_length = std::max(reached_arc_length, (theta - theta_goal) * lane_radius);
  }
  // Measure from the goal so the first swept segment (goal -> first extended point) counts too.
  const double lateral_distance = distance_to_polyline(result, points.size() - 1, pedestrian);
  std::cout << "[scenario] arc length goal -> pedestrian = " << s_to_pedestrian << " m\n"
            << "[scenario] extension reaches             = " << reached_arc_length << " m\n"
            << "[scenario] pedestrian to swept path      = " << lateral_distance << " m\n"
            << "[scenario] pointcloud lateral reach      = " << pointcloud_reach << " m"
            << std::endl;
  // The extension covers the pedestrian's arc position ...
  EXPECT_GT(reached_arc_length, s_to_pedestrian);
  // ... and the swept footprint reaches it.
  EXPECT_LT(lateral_distance, pointcloud_reach);
}

// The curvature that drives the extension is measured on the untrimmed trajectory, so it must
// survive the ego getting close to the goal. Trim an arc down to what is left a short distance
// before the goal and check that the extension still follows the road: decimating that remainder
// at step_length leaves too few points to define a circle, which is what made the estimate
// collapse to zero when it was taken from the decimated points.
TEST(MvpUtilsDecimate, ExtensionKeepsFollowingCurvatureNearTheGoal)
{
  constexpr double lane_radius = 5.0;
  constexpr double extend_distance = 6.0;
  constexpr double step_length = 2.0;
  // 1.8 m of trajectory left: below the ~2.0 m where decimation stops yielding three points.
  for (const double remaining : {6.0, 3.0, 1.8, 1.0}) {
    const auto points =
      make_arc_forward_trajectory(static_cast<size_t>(remaining / 0.25) + 1, 0.25, lane_radius);
    geometry_msgs::msg::Pose ego_pose = points.front().pose;
    const auto result =
      autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
        points, ego_pose, 3.0, 1.046, step_length, extend_distance);
    ASSERT_GE(result.size(), 2u);
    // The tail of the output is the extension; every extended point must stay on the arc.
    double worst_drift = 0.0;
    for (const auto & p : result) {
      const auto & q = p.pose.position;
      worst_drift =
        std::max(worst_drift, std::abs(std::hypot(q.x, q.y - lane_radius) - lane_radius));
    }
    std::cout << "[near-goal] remaining = " << remaining << " m, points=" << result.size()
              << std::endl;
    std::cout << "[near-goal] remaining = " << remaining << " m -> worst drift = " << worst_drift
              << " m" << std::endl;
    EXPECT_LT(worst_drift, 1e-6) << "remaining = " << remaining << " m";
  }
}

// An explicit curvature overrides the estimate taken from the input points. This is the path
// decimate_trajectory_points_from_ego() uses to hand down the curvature it measured upstream.
TEST(MvpUtilsExtend, ExplicitCurvatureOverridesTheEstimate)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // estimate would be zero
  constexpr double radius = 5.0;
  const auto result = get_extended_trajectory_points(points, 6.0, 2.0, 1.0 / radius);
  ASSERT_GT(result.size(), points.size());
  // With a curvature of 1/5 the extension bends away from the straight input instead of
  // continuing along it.
  const auto & last = result.back().pose.position;
  const double straight_x = points.back().pose.position.x + 6.0;
  EXPECT_LT(last.x, straight_x - 0.5);
  EXPECT_GT(std::abs(last.y), 0.5);
}

// Driving backwards extends behind the goal. isDrivingForwardWithTwist() decides the direction
// from the geometry of the first two points once there is more than one, so the poses -- not the
// velocity sign -- are what make this a reversing trajectory.
TEST(MvpUtilsExtend, BackwardDrivingExtendsBehindAlongTheArc)
{
  constexpr double radius = 5.0;
  const auto forward = make_arc_forward_trajectory(6, 1.0, radius);
  // Same path, traversed the other way. A reversing vehicle still faces forwards, so the poses
  // keep the original arc tangent and only the order and the velocity sign flip. Rotating the
  // poses as well would cancel out and read as driving forwards again.
  std::vector<TrajectoryPoint> points(forward.rbegin(), forward.rend());
  for (auto & p : points) {
    p.longitudinal_velocity_mps = -1.0;
  }
  ASSERT_FALSE(autoware::motion_utils::isDrivingForwardWithTwist(points).value_or(true));
  const auto result = get_extended_trajectory_points(points, 6.0, 2.0);
  ASSERT_GT(result.size(), points.size());
  // The goal is now the start of the original arc; the extension runs past it, still on the arc.
  const double goal_arc = 0.0;
  for (size_t i = points.size(); i < result.size(); ++i) {
    const auto & q = result.at(i).pose.position;
    EXPECT_LT(std::abs(std::hypot(q.x, q.y - radius) - radius), 1e-6) << "point " << i;
    EXPECT_LT(std::atan2(q.x, radius - q.y), goal_arc) << "point " << i;
  }
}

// ----------------------------- calc_distance_to_front_object -----------------------------------

TEST(MvpUtilsFrontObject, ReturnsArcLengthForObjectAhead)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // x = 0..4
  // ego at index 1 (x = 1), obstacle ahead at x = 3.2 (nearest index 3).
  const auto dist = calc_distance_to_front_object(points, 1, make_point(3.2, 0.0));
  ASSERT_TRUE(dist.has_value());
  // signed arc length from index 1 (x=1) to nearest index of obstacle (x=3) is 2.0.
  EXPECT_NEAR(*dist, 2.0, 1e-6);
}

TEST(MvpUtilsFrontObject, ReturnsNulloptForObjectBehind)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // x = 0..4
  // ego at index 3 (x = 3), obstacle behind at x = 0.2 (nearest index 0) -> negative arc length.
  const auto dist = calc_distance_to_front_object(points, 3, make_point(0.2, 0.0));
  EXPECT_FALSE(dist.has_value());
}

TEST(MvpUtilsFrontObject, EmptyTrajectoryThrows)
{
  // Precondition violation: findNearestIndex validates a non-empty trajectory and throws on an
  // empty one. Pin that the precondition is enforced rather than silently producing a result.
  const std::vector<TrajectoryPoint> empty_points;
  EXPECT_THROW(
    calc_distance_to_front_object(empty_points, 0, make_point(1.0, 0.0)), std::invalid_argument);
}

// ----------------------------- concat_vectors --------------------------------------------------

TEST(MvpUtilsConcat, ConcatenatesInOrder)
{
  const std::vector<int> a{1, 2, 3};
  const std::vector<int> b{4, 5};
  const auto result = concat_vectors(a, b);
  const std::vector<int> expected{1, 2, 3, 4, 5};
  EXPECT_EQ(result, expected);
}

TEST(MvpUtilsConcat, HandlesEmptyInputs)
{
  const std::vector<int> empty;
  const std::vector<int> b{7, 8};
  EXPECT_EQ(concat_vectors(empty, b), b);
  EXPECT_EQ(concat_vectors(b, empty), b);
  EXPECT_TRUE(concat_vectors(empty, empty).empty());
}

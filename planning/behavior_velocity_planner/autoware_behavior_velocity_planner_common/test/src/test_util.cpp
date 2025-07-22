// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "utils.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

using namespace autoware::behavior_velocity_planner;                  // NOLINT
using namespace autoware::behavior_velocity_planner::planning_utils;  // NOLINT
using autoware_planning_msgs::msg::PathPoint;

TEST(PlanningUtilsTest, calcSegmentIndexFromPointIndex)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  geometry_msgs::msg::Point point;
  point.x = 4.5;
  point.y = 0.0;

  size_t result = calcSegmentIndexFromPointIndex(path.points, point, 4);

  EXPECT_EQ(result, 4);
}

TEST(PlanningUtilsTest, calculateOffsetPoint2d)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));  // No rotation

  double offset_x = 1.0;
  double offset_y = 1.0;

  auto result = calculateOffsetPoint2d(pose, offset_x, offset_y);

  EXPECT_NEAR(result.x(), 1.0, 0.1);
  EXPECT_NEAR(result.y(), 1.0, 0.1);
}

TEST(PlanningUtilsTest, createDetectionAreaPolygons)
{
  // using namespace autoware::behavior_velocity_planner::planning_utils;

  // Input parameters
  Polygons2d da_polys;
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  geometry_msgs::msg::Pose target_pose;
  size_t target_seg_idx = 0;
  autoware::behavior_velocity_planner::DetectionRange da_range;

  da_range.min_longitudinal_distance = 1.0;
  da_range.max_longitudinal_distance = 10.0;
  da_range.max_lateral_distance = 2.0;
  da_range.interval = 5.0;
  da_range.wheel_tread = 1.0;
  da_range.left_overhang = 0.5;
  da_range.right_overhang = 0.5;
  da_range.use_left = true;
  da_range.use_right = true;

  double obstacle_vel_mps = 0.5;
  double min_velocity = 1.0;

  // Path with some points
  for (double i = 0.0; i < 3.0; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = i * 5.0;
    point.point.pose.position.y = 0.0;
    point.point.longitudinal_velocity_mps = 1.0;
    path.points.push_back(point);
  }

  // Target pose
  target_pose.position.x = 0.0;
  target_pose.position.y = 0.0;

  // Call the function
  bool success = createDetectionAreaPolygons(
    da_polys, path, target_pose, target_seg_idx, da_range, obstacle_vel_mps, min_velocity);

  // Assert success
  EXPECT_TRUE(success);

  // Validate results
  ASSERT_FALSE(da_polys.empty());
  EXPECT_EQ(da_polys.size(), 2);  // Expect polygons for left and right bounds

  // Check the first polygon
  auto & polygon = da_polys.front();
  EXPECT_EQ(polygon.outer().size(), 7);  // Each polygon should be a rectangle

  // Check some specific points
  EXPECT_NEAR(polygon.outer()[0].x(), 1.0, 0.1);  // Left inner bound
  EXPECT_NEAR(polygon.outer()[0].y(), 1.0, 0.1);
}

// Test for calcJudgeLineDistWithAccLimit
TEST(PlanningUtilsTest, calcJudgeLineDistWithAccLimit)
{
  double velocity = 10.0;               // m/s
  double max_stop_acceleration = -3.0;  // m/s^2
  double delay_response_time = 1.0;     // s

  double result =
    calcJudgeLineDistWithAccLimit(velocity, max_stop_acceleration, delay_response_time);

  EXPECT_NEAR(result, 26.67, 0.01);  // Updated expected value
}

// Test for calcJudgeLineDistWithJerkLimit
TEST(PlanningUtilsTest, calcJudgeLineDistWithJerkLimit)
{
  double velocity = 10.0;               // m/s
  double acceleration = 0.0;            // m/s^2
  double max_stop_acceleration = -3.0;  // m/s^2
  double max_stop_jerk = -1.0;          // m/s^3
  double delay_response_time = 1.0;     // s

  double result = calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_stop_acceleration, max_stop_jerk, delay_response_time);

  EXPECT_GT(result, 0.0);  // The result should be positive
}

// Test for isAheadOf
TEST(PlanningUtilsTest, isAheadOf)
{
  geometry_msgs::msg::Pose target;
  geometry_msgs::msg::Pose origin;
  target.position.x = 10.0;
  target.position.y = 0.0;
  origin.position.x = 0.0;
  origin.position.y = 0.0;
  origin.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));  // No rotation

  EXPECT_TRUE(isAheadOf(target, origin));

  target.position.x = -10.0;
  EXPECT_FALSE(isAheadOf(target, origin));
}

TEST(PlanningUtilsTest, insertDecelPoint)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  geometry_msgs::msg::Point stop_point;
  stop_point.x = 5.0;
  stop_point.y = 0.0;

  auto stop_pose = insertDecelPoint(stop_point, path, 5.0);
  ASSERT_TRUE(stop_pose.has_value());
  EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
}

// Test for insertVelocity
TEST(PlanningUtilsTest, insertVelocity)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
  path_point.point.pose.position.x = 5.0;
  path_point.point.pose.position.y = 0.0;
  path_point.point.longitudinal_velocity_mps = 10.0;

  size_t insert_index = 5;
  insertVelocity(path, path_point, 10.0, insert_index);

  EXPECT_EQ(path.points.size(), 11);
  EXPECT_NEAR(path.points.at(insert_index).point.longitudinal_velocity_mps, 10.0, 0.1);
}

// Test for insertStopPoint
TEST(PlanningUtilsTest, insertStopPoint)
{
  {
    auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
    geometry_msgs::msg::Point stop_point;
    stop_point.x = 5.0;
    stop_point.y = 0.0;

    auto stop_pose = insertStopPoint(stop_point, path);
    ASSERT_TRUE(stop_pose.has_value());
    EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
  }
  {
    auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
    geometry_msgs::msg::Point stop_point;
    stop_point.x = 5.0;
    stop_point.y = 0.0;

    auto stop_pose = insertStopPoint(stop_point, 4, path);
    ASSERT_TRUE(stop_pose.has_value());
    EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
  }
}

// Test for getAheadPose
TEST(PlanningUtilsTest, getAheadPose)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point1;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point2;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point3;
  point1.point.pose.position.x = 0.0;
  point2.point.pose.position.x = 5.0;
  point3.point.pose.position.x = 10.0;

  path.points.emplace_back(point1);
  path.points.emplace_back(point2);
  path.points.emplace_back(point3);

  double ahead_dist = 7.0;
  auto pose = getAheadPose(0, ahead_dist, path);

  EXPECT_NEAR(pose.position.x, 7.0, 0.1);
}

TEST(PlanningUtilsTest, calcDecelerationVelocityFromDistanceToTarget)
{
  double max_slowdown_jerk = -1.0;    // m/s^3
  double max_slowdown_accel = -3.0;   // m/s^2
  double current_accel = -1.0;        // m/s^2
  double current_velocity = 10.0;     // m/s
  double distance_to_target = 100.0;  // m

  double result = calcDecelerationVelocityFromDistanceToTarget(
    max_slowdown_jerk, max_slowdown_accel, current_accel, current_velocity, distance_to_target);

  EXPECT_LT(result, current_velocity);
}

// Test for toRosPoints
TEST(PlanningUtilsTest, ToRosPoints)
{
  using autoware_perception_msgs::msg::PredictedObject;
  PredictedObjects objects;

  // Add a predicted object
  PredictedObject obj1;
  obj1.kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  obj1.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  obj1.kinematics.initial_pose_with_covariance.pose.position.z = 3.0;
  objects.objects.push_back(obj1);

  // Add another predicted object
  PredictedObject obj2;
  obj2.kinematics.initial_pose_with_covariance.pose.position.x = 4.0;
  obj2.kinematics.initial_pose_with_covariance.pose.position.y = 5.0;
  obj2.kinematics.initial_pose_with_covariance.pose.position.z = 6.0;
  objects.objects.push_back(obj2);

  auto points = toRosPoints(objects);

  ASSERT_EQ(points.size(), 2);  // Verify the number of points
  EXPECT_EQ(points[0].x, 1.0);
  EXPECT_EQ(points[0].y, 2.0);
  EXPECT_EQ(points[0].z, 3.0);
  EXPECT_EQ(points[1].x, 4.0);
  EXPECT_EQ(points[1].y, 5.0);
  EXPECT_EQ(points[1].z, 6.0);
}

// Test for extendSegmentToBounds
TEST(PlanningUtilsTest, ExtendSegmentToBounds)
{
  constexpr auto epsilon = 1e-3;
  constexpr auto make_bound = [](const Point2d & start, const Point2d & end) {
    return std::vector{
      geometry_msgs::msg::Point{}.set__x(start.x()).set__y(start.y()),
      geometry_msgs::msg::Point{}.set__x(end.x()).set__y(end.y())};
  };

  const auto bound1 = make_bound({-1.0, -1.0}, {1.0, -1.0});
  const auto bound2 = make_bound({1.0, 2.0}, {2.0, 1.0});

  {  // normal case
    const lanelet::BasicLineString2d segment{{0.0, 0.0}, {1.0, 1.0}};

    const auto result = extendSegmentToBounds(segment, bound1, bound2);

    ASSERT_EQ(result.size(), 2);  // Verify the segment has two points

    // Check the output segment coordinates
    EXPECT_NEAR(result[0].x(), -1.0, epsilon);
    EXPECT_NEAR(result[0].y(), -1.0, epsilon);
    EXPECT_NEAR(result[1].x(), 1.5, epsilon);
    EXPECT_NEAR(result[1].y(), 1.5, epsilon);
  }

  {  // input segment is reversed
    const lanelet::BasicLineString2d segment{{1.0, 1.0}, {0.0, 0.0}};

    const auto result = extendSegmentToBounds(segment, bound1, bound2);

    ASSERT_EQ(result.size(), 2);  // Verify the segment has two points

    // Check the output segment coordinates
    EXPECT_NEAR(result[0].x(), 1.5, epsilon);
    EXPECT_NEAR(result[0].y(), 1.5, epsilon);
    EXPECT_NEAR(result[1].x(), -1.0, epsilon);
    EXPECT_NEAR(result[1].y(), -1.0, epsilon);
  }

  {  // input segment is empty
    const lanelet::BasicLineString2d segment{};

    const auto result = extendSegmentToBounds(segment, bound1, bound2);

    ASSERT_EQ(result.size(), 0);  // Verify the segment is empty
  }

  {  // input segment has more than 2 points
    const lanelet::BasicLineString2d segment{{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};

    const auto result = extendSegmentToBounds(segment, bound1, bound2);

    ASSERT_EQ(result.size(), 3);  // Verify the segment is returned as is

    // Check the output segment coordinates
    EXPECT_NEAR(result[0].x(), segment[0].x(), epsilon);
    EXPECT_NEAR(result[0].y(), segment[0].y(), epsilon);
    EXPECT_NEAR(result[1].x(), segment[1].x(), epsilon);
    EXPECT_NEAR(result[1].y(), segment[1].y(), epsilon);
    EXPECT_NEAR(result[2].x(), segment[2].x(), epsilon);
    EXPECT_NEAR(result[2].y(), segment[2].y(), epsilon);
  }

  {  // input segment does not intersect bounds
    const lanelet::BasicLineString2d segment{{-1.0, 0.0}, {1.0, 0.0}};

    const auto result = extendSegmentToBounds(segment, bound1, bound2);

    ASSERT_EQ(result.size(), 2);  // Verify the segment is returned as is

    // Check the output segment coordinates
    EXPECT_NEAR(result[0].x(), segment[0].x(), epsilon);
    EXPECT_NEAR(result[0].y(), segment[0].y(), epsilon);
    EXPECT_NEAR(result[1].x(), segment[1].x(), epsilon);
    EXPECT_NEAR(result[1].y(), segment[1].y(), epsilon);
  }

  {  // extended segment has no overlap with original segment
    const lanelet::BasicLineString2d segment{{0.0, 1.0}, {0.0, -1.0}};

    const auto bound1_excluding_segment = make_bound({-1.0, 2.0}, {1.0, 2.0});
    const auto bound2_excluding_segment = make_bound({-1.0, 3.0}, {1.0, 3.0});

    const auto result =
      extendSegmentToBounds(segment, bound1_excluding_segment, bound2_excluding_segment);

    ASSERT_EQ(result.size(), 2);

    // Check the output segment coordinates (should be original segment)
    EXPECT_NEAR(result[0].x(), segment[0].x(), epsilon);
    EXPECT_NEAR(result[0].y(), segment[0].y(), epsilon);
    EXPECT_NEAR(result[1].x(), segment[1].x(), epsilon);
    EXPECT_NEAR(result[1].y(), segment[1].y(), epsilon);
  }

  {  // extended segment has no overlap with original segment (negative y direction)
    const lanelet::BasicLineString2d segment{{0.0, 1.0}, {0.0, -1.0}};

    const auto bound1_excluding_segment = make_bound({-1.0, -2.0}, {1.0, -2.0});
    const auto bound2_excluding_segment = make_bound({-1.0, -3.0}, {1.0, -3.0});

    const auto result =
      extendSegmentToBounds(segment, bound1_excluding_segment, bound2_excluding_segment);

    ASSERT_EQ(result.size(), 2);

    // Check the output segment coordinates (should be original segment)
    EXPECT_NEAR(result[0].x(), segment[0].x(), epsilon);
    EXPECT_NEAR(result[0].y(), segment[0].y(), epsilon);
    EXPECT_NEAR(result[1].x(), segment[1].x(), epsilon);
    EXPECT_NEAR(result[1].y(), segment[1].y(), epsilon);
  }

  {  // extended segment within the boundary of the adjacent lane
    //
    // ======================================
    //      ^
    //      |
    //      | segment (original line)
    //      |
    //      v
    // ====================================== boundary1
    //      ^
    //      | extended line (this does not have any overlap with the original segment,
    //      |                so the original segment should be returned)
    //      v
    // ====================================== boundary2

    const lanelet::BasicLineString2d segment{{0.0, 1.0}, {0.0, -1.0}};

    const auto bound1_excluding_segment = make_bound({-1.0, -1.0}, {1.0, -1.0});
    const auto bound2_excluding_segment = make_bound({-1.0, -3.0}, {1.0, -3.0});

    const auto result =
      extendSegmentToBounds(segment, bound1_excluding_segment, bound2_excluding_segment);

    ASSERT_EQ(result.size(), 2);

    // Check the output segment coordinates (should be original segment)
    EXPECT_NEAR(result[0].x(), segment[0].x(), epsilon);
    EXPECT_NEAR(result[0].y(), segment[0].y(), epsilon);
    EXPECT_NEAR(result[1].x(), segment[1].x(), epsilon);
    EXPECT_NEAR(result[1].y(), segment[1].y(), epsilon);
  }
}

TEST(PlanningUtilsTest, getConstLaneletsFromIds)
{
  const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  lanelet::LaneletMapPtr map =
    autoware::test_utils::loadMap(package_dir + "/test_map/lanelet2_map.osm");

  auto lanelets = getConstLaneletsFromIds(map, {10333, 10310, 10291});

  EXPECT_EQ(lanelets.size(), 3);
}

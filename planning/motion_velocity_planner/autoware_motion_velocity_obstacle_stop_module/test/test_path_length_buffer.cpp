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

#include "path_length_buffer.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <functional>
#include <optional>

namespace autoware::motion_velocity_planner::obstacle_stop
{
namespace
{

// Identity-style stop-distance callback: the buffer recomputes each item's distance from its stored
// stop_point, so encoding the distance in stop_point.x makes the synthetic timeline deterministic.
std::function<double(const geometry_msgs::msg::Point &)> distance_from_x()
{
  return [](const geometry_msgs::msg::Point & p) { return p.x; };
}

geometry_msgs::msg::Point point_at(const double x)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  return p;
}

StopObstacle make_stop_obstacle(const double desired_margin = 0.0)
{
  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  const StopObstacleClassification classification(StopObstacleClassification::Type::POINTCLOUD);
  PolygonParam polygon_param;
  return StopObstacle(
    stamp, classification, /*lon_velocity=*/0.0, /*collision_point=*/point_at(0.0),
    /*dist_to_collide_on_decimated_traj=*/0.0, polygon_param, /*braking_dist=*/desired_margin);
}

rclcpp::Time t(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

}  // namespace

TEST(PathLengthBuffer, EmptyBufferHasNoActiveItem)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/0.5, /*min_on=*/1.0);
  EXPECT_FALSE(buffer.get_nearest_active_item().has_value());
}

TEST(PathLengthBuffer, NulloptStopPointIsNoOp)
{
  PathLengthBuffer buffer(1.0, 0.5, 1.0);
  buffer.update_buffer(std::nullopt, distance_from_x(), t(0.0), make_stop_obstacle(), 0.0);
  EXPECT_FALSE(buffer.get_nearest_active_item().has_value());
}

TEST(PathLengthBuffer, ActivatesOnlyAfterMinOnDuration)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/10.0, /*min_on=*/1.0);

  // First observation creates an inactive item; it is not yet returned.
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.0), make_stop_obstacle(), 0.0);
  EXPECT_FALSE(buffer.get_nearest_active_item().has_value());

  // Still within min_on_duration -> remains inactive.
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.5), make_stop_obstacle(), 0.0);
  EXPECT_FALSE(buffer.get_nearest_active_item().has_value());

  // Past min_on_duration -> becomes active.
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(2.0), make_stop_obstacle(), 0.0);
  const auto item = buffer.get_nearest_active_item();
  ASSERT_TRUE(item.has_value());
  EXPECT_DOUBLE_EQ(item->stop_distance, 10.0);
}

TEST(PathLengthBuffer, EvictsActiveItemAfterMinOffDuration)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/0.5, /*min_on=*/1.0);

  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.0), make_stop_obstacle(), 0.0);
  // Activate the item (and reset its start_time to t=2.0).
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(2.0), make_stop_obstacle(), 0.0);
  ASSERT_TRUE(buffer.get_nearest_active_item().has_value());

  // A far-away stop point is not matched, so the active item is not refreshed; with elapsed time
  // beyond min_off_duration it is evicted, and the new (inactive) item is not yet returned.
  buffer.update_buffer(point_at(100.0), distance_from_x(), t(3.0), make_stop_obstacle(), 0.0);
  EXPECT_FALSE(buffer.get_nearest_active_item().has_value());
}

TEST(PathLengthBuffer, EvictsInactiveItemBeyondUpdateDistanceThreshold)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/10.0, /*min_on=*/1.0);

  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.0), make_stop_obstacle(), 0.0);
  // |10 - 100| > update_distance_threshold while still inactive -> the first item is evicted and a
  // new inactive item (distance 100) replaces it.
  buffer.update_buffer(point_at(100.0), distance_from_x(), t(0.1), make_stop_obstacle(), 0.0);

  // After enough time the surviving item activates; its distance proves the old item was evicted
  // (otherwise the nearest active distance would be 10, not 100).
  buffer.update_buffer(point_at(100.0), distance_from_x(), t(2.0), make_stop_obstacle(), 0.0);
  const auto item = buffer.get_nearest_active_item();
  ASSERT_TRUE(item.has_value());
  EXPECT_DOUBLE_EQ(item->stop_distance, 100.0);
}

TEST(PathLengthBuffer, PrefersActiveItemOverNearerInactiveItem)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/10.0, /*min_on=*/1.0);

  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.0), make_stop_obstacle(), 0.0);
  // Re-observe the same item; it stays inactive because should_activate() requires the elapsed time
  // to be strictly greater than min_on (1.0), and here it is exactly 1.0.
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(1.0), make_stop_obstacle(), 0.0);
  // This update activates the distance-10 item (elapsed 2.0 > min_on) and appends a second, nearer
  // (distance 5) but inactive item (|10 - 5| > threshold, so it is appended rather than merged).
  buffer.update_buffer(point_at(5.0), distance_from_x(), t(2.0), make_stop_obstacle(), 0.0);

  // The comparator must prefer the active item even though the inactive one is nearer.
  const auto item = buffer.get_nearest_active_item();
  ASSERT_TRUE(item.has_value());
  EXPECT_DOUBLE_EQ(item->stop_distance, 10.0);
}

TEST(PathLengthBuffer, UpdatesNearestItemWhenRelativeDistancePositive)
{
  PathLengthBuffer buffer(/*update_distance_threshold=*/1.0, /*min_off=*/10.0, /*min_on=*/1.0);

  buffer.update_buffer(point_at(10.0), distance_from_x(), t(0.0), make_stop_obstacle(0.1), 0.0);
  // Re-observe the item at distance 10 with margin 0.1; it stays inactive because should_activate()
  // requires the elapsed time to be strictly greater than min_on (1.0), and here it is exactly 1.0.
  buffer.update_buffer(point_at(10.0), distance_from_x(), t(1.0), make_stop_obstacle(0.1), 0.0);

  // New observation within threshold and nearer (rel_dist = 10 - 9.5 = 0.5 > 0). Elapsed 2.0 >
  // min_on, so the item activates here and its stop_distance and determined obstacle/margin are
  // refreshed.
  buffer.update_buffer(point_at(9.5), distance_from_x(), t(2.0), make_stop_obstacle(0.9), 2.5);

  const auto item = buffer.get_nearest_active_item();
  ASSERT_TRUE(item.has_value());
  EXPECT_DOUBLE_EQ(item->stop_distance, 9.5);
  EXPECT_DOUBLE_EQ(item->determined_desired_stop_margin, 2.5);
  ASSERT_TRUE(item->determined_stop_obstacle.braking_dist.has_value());
  EXPECT_DOUBLE_EQ(item->determined_stop_obstacle.braking_dist.value(), 0.9);
}

}  // namespace autoware::motion_velocity_planner::obstacle_stop

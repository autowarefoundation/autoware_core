// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_velocity_planner_common/collision_checker.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <random>
#include <vector>

using autoware::motion_velocity_planner::CollisionChecker;
using autoware_utils_geometry::Line2d;
using autoware_utils_geometry::MultiLineString2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

Point2d random_point()
{
  static std::random_device r;
  static std::default_random_engine e1(r());
  static std::uniform_real_distribution<double> uniform_dist(-100, 100);
  return {uniform_dist(e1), uniform_dist(e1)};
}

Line2d random_line()
{
  const auto point = random_point();
  const auto point2 = Point2d{point.x() + 1, point.y() + 1};
  const auto point3 = Point2d{point2.x() - 1, point2.y() + 1};
  const auto point4 = Point2d{point3.x() + 1, point3.y() + 1};
  return {point, point2, point3, point4};
}

Polygon2d random_polygon()
{
  Polygon2d polygon;
  const auto point = random_point();
  const auto point2 = Point2d{point.x() + 1, point.y() + 4};
  const auto point3 = Point2d{point.x() + 4, point.y() + 4};
  const auto point4 = Point2d{point.x() + 3, point.y() + 1};
  polygon.outer() = {point, point2, point3, point4, point};
  return polygon;
}

bool all_within(const MultiPoint2d & pts1, const MultiPoint2d & pts2)
{
  // results from the collision checker and the direct checks can have some small precision errors
  constexpr auto eps = 1e-2;
  for (const auto & p1 : pts1) {
    bool found = false;
    for (const auto & p2 : pts2) {
      if (boost::geometry::comparable_distance(p1, p2) < eps) {
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

TEST(TestCollisionChecker, DISABLED_Benchmark)
{
  constexpr auto nb_ego_footprints = 1000;
  constexpr auto nb_obstacles = 1000;
  MultiPolygon2d ego_footprints;
  ego_footprints.reserve(nb_ego_footprints);
  for (auto i = 0; i < nb_ego_footprints; ++i) {
    ego_footprints.push_back(random_polygon());
  }
  const auto cc_constr_start = std::chrono::system_clock::now();
  CollisionChecker collision_checker(ego_footprints);
  const auto cc_constr_end = std::chrono::system_clock::now();
  const auto cc_constr_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(cc_constr_end - cc_constr_start).count();
  std::printf(
    "Collision checker construction (with %d footprints): %ld ns\n", nb_ego_footprints,
    cc_constr_ns);
  MultiPolygon2d poly_obstacles;
  MultiPoint2d point_obstacles;
  MultiLineString2d line_obstacles;
  for (auto i = 0; i < nb_obstacles; ++i) {
    poly_obstacles.push_back(random_polygon());
    point_obstacles.push_back(random_point());
    line_obstacles.push_back(random_line());
  }
  const auto check_obstacles_one_by_one = [&](const auto & obstacles) {
    std::chrono::nanoseconds collision_checker_ns{};
    std::chrono::nanoseconds naive_ns{};
    for (const auto & obs : obstacles) {
      const auto cc_start = std::chrono::system_clock::now();
      const auto collisions = collision_checker.get_collisions(obs);
      MultiPoint2d cc_collision_points;
      for (const auto & c : collisions)
        cc_collision_points.insert(
          cc_collision_points.end(), c.collision_points.begin(), c.collision_points.end());
      const auto cc_end = std::chrono::system_clock::now();
      const auto naive_start = std::chrono::system_clock::now();
      MultiPoint2d naive_collision_points;
      for (const auto & ego_footprint : ego_footprints) {
        MultiPoint2d points;
        boost::geometry::intersection(ego_footprint, obs, points);
        naive_collision_points.insert(naive_collision_points.end(), points.begin(), points.end());
      }
      const auto naive_end = std::chrono::system_clock::now();
      const auto equal = all_within(cc_collision_points, naive_collision_points) &&
                         all_within(naive_collision_points, cc_collision_points);
      EXPECT_TRUE(equal);
      if (!equal) {
        std::cout << "cc: " << boost::geometry::wkt(cc_collision_points) << std::endl;
        std::cout << "naive: " << boost::geometry::wkt(naive_collision_points) << std::endl;
      }
      collision_checker_ns +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(cc_end - cc_start);
      naive_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(naive_end - naive_start);
    }
    std::printf("%20s%10ld ns\n", "collision checker : ", collision_checker_ns.count());
    std::printf("%20s%10ld ns\n", "naive : ", naive_ns.count());
  };
  const auto check_obstacles = [&](const auto & obstacles) {
    std::chrono::nanoseconds collision_checker_ns{};
    std::chrono::nanoseconds naive_ns{};
    const auto cc_start = std::chrono::system_clock::now();
    const auto collisions = collision_checker.get_collisions(obstacles);
    MultiPoint2d cc_collision_points;
    for (const auto & c : collisions)
      cc_collision_points.insert(
        cc_collision_points.end(), c.collision_points.begin(), c.collision_points.end());
    const auto cc_end = std::chrono::system_clock::now();
    const auto naive_start = std::chrono::system_clock::now();
    MultiPoint2d naive_collision_points;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    // on NVIDIA DRIVE AGX Thor, boost::geometry triggers a false positive warning
    boost::geometry::intersection(ego_footprints, obstacles, naive_collision_points);
#pragma GCC diagnostic pop

    const auto naive_end = std::chrono::system_clock::now();
    const auto equal = all_within(cc_collision_points, naive_collision_points) &&
                       all_within(naive_collision_points, cc_collision_points);
    EXPECT_TRUE(equal);
    if (!equal) {
      std::cout << "cc: " << boost::geometry::wkt(cc_collision_points) << std::endl;
      std::cout << "naive: " << boost::geometry::wkt(naive_collision_points) << std::endl;
    }
    collision_checker_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(cc_end - cc_start);
    naive_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(naive_end - naive_start);
    std::printf("%20s%10ld ns\n", "collision checker : ", collision_checker_ns.count());
    std::printf("%20s%10ld ns\n", "naive : ", naive_ns.count());
  };

  std::cout << "* check one by one\n";
  std::printf("%d Polygons:\n", nb_obstacles);
  check_obstacles_one_by_one(poly_obstacles);
  std::printf("%d Lines:\n", nb_obstacles);
  check_obstacles_one_by_one(line_obstacles);
  std::printf("%d Points:\n", nb_obstacles);
  check_obstacles_one_by_one(point_obstacles);
  std::cout << "* check all at once\n";
  std::printf("%d Polygons:\n", nb_obstacles);
  check_obstacles(poly_obstacles);
  std::printf("%d Lines:\n", nb_obstacles);
  check_obstacles(line_obstacles);
  std::printf("%d Points:\n", nb_obstacles);
  check_obstacles(point_obstacles);
}

namespace
{
// Build an axis-aligned unit square footprint with the given lower-left corner.
Polygon2d make_square(const double x, const double y, const double size = 1.0)
{
  Polygon2d polygon;
  polygon.outer() = {
    Point2d{x, y}, Point2d{x, y + size}, Point2d{x + size, y + size}, Point2d{x + size, y},
    Point2d{x, y}};
  return polygon;
}
}  // namespace

TEST(TestCollisionChecker, TrajectorySizeAndRtreePopulatedFromFootprints)
{
  MultiPolygon2d footprints;
  footprints.push_back(make_square(0.0, 0.0));
  footprints.push_back(make_square(10.0, 0.0));
  CollisionChecker collision_checker(footprints);

  EXPECT_EQ(collision_checker.trajectory_size(), 2u);
  ASSERT_NE(collision_checker.get_rtree(), nullptr);
  EXPECT_EQ(collision_checker.get_rtree()->size(), 2u);
}

TEST(TestCollisionChecker, EmptyFootprintsReportsNoCollision)
{
  const MultiPolygon2d empty_footprints;
  CollisionChecker collision_checker(empty_footprints);

  EXPECT_EQ(collision_checker.trajectory_size(), 0u);
  const Point2d obstacle{0.5, 0.5};
  EXPECT_TRUE(collision_checker.get_collisions(obstacle).empty());
}

TEST(TestCollisionChecker, PointInsideSingleFootprint)
{
  MultiPolygon2d footprints;
  footprints.push_back(make_square(0.0, 0.0));   // index 0: [0,1]x[0,1]
  footprints.push_back(make_square(10.0, 0.0));  // index 1: far away
  CollisionChecker collision_checker(footprints);

  const Point2d inside{0.5, 0.5};
  const auto collisions = collision_checker.get_collisions(inside);
  ASSERT_EQ(collisions.size(), 1u);
  EXPECT_EQ(collisions.front().trajectory_index, 0u);
  ASSERT_EQ(collisions.front().collision_points.size(), 1u);
  EXPECT_NEAR(collisions.front().collision_points.front().x(), 0.5, 1e-9);
  EXPECT_NEAR(collisions.front().collision_points.front().y(), 0.5, 1e-9);
}

TEST(TestCollisionChecker, PointOutsideAllFootprints)
{
  MultiPolygon2d footprints;
  footprints.push_back(make_square(0.0, 0.0));
  footprints.push_back(make_square(10.0, 0.0));
  CollisionChecker collision_checker(footprints);

  const Point2d outside{5.0, 5.0};
  EXPECT_TRUE(collision_checker.get_collisions(outside).empty());
}

TEST(TestCollisionChecker, LineCrossingMultipleFootprints)
{
  MultiPolygon2d footprints;
  footprints.push_back(make_square(0.0, 0.0));   // index 0: [0,1]x[0,1]
  footprints.push_back(make_square(2.0, 0.0));   // index 1: [2,3]x[0,1]
  footprints.push_back(make_square(10.0, 0.0));  // index 2: far away
  CollisionChecker collision_checker(footprints);

  // Horizontal line at y = 0.5 from x = -1 to x = 4 crosses footprints 0 and 1 only.
  const Line2d line{Point2d{-1.0, 0.5}, Point2d{4.0, 0.5}};
  const auto collisions = collision_checker.get_collisions(line);
  ASSERT_EQ(collisions.size(), 2u);

  std::vector<size_t> hit_indices;
  for (const auto & c : collisions) {
    hit_indices.push_back(c.trajectory_index);
    EXPECT_FALSE(c.collision_points.empty());
  }
  std::sort(hit_indices.begin(), hit_indices.end());
  EXPECT_EQ(hit_indices, (std::vector<size_t>{0u, 1u}));
}

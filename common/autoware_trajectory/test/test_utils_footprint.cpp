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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/footprint.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace
{
using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;

static Trajectory<geometry_msgs::msg::Pose> build_parabolic_trajectory(
  const size_t num_points, const double interval, const bool reverse = false)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  double half = static_cast<double>(num_points - 1) / 2.0;
  for (size_t i = 0; i < num_points; ++i) {
    double x;
    double y;
    double yaw;
    if (!reverse) {
      x = (static_cast<double>(i) - half) * interval;
      y = x * x;                       // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0);  // Tangent direction
    } else {
      x = (-1 * static_cast<double>(i) + half) * interval;
      y = x * x;                              // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0) - M_PI;  // Tangent direction
    }
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
    raw_poses.push_back(p);
  }
  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

// Test 1: check type of long path polygon from build_path_polygon
TEST(trajectoryFootprint, LongPolygon)
{
  auto traj = build_parabolic_trajectory(11, 0.5);
  const auto polygon = autoware::experimental::trajectory::build_path_polygon(
    traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 1.0, 0.5);

  EXPECT_EQ(typeid(polygon), typeid(autoware_utils_geometry::Polygon2d))
    << "polygon is not autoware_utils_geometry::Polygon2d.";
}

// Test 2: count numbers of footprint trace with default interval
TEST(trajectoryFootprint, FootprintTraceDefault)
{
  autoware_utils_geometry::Point2d left_front{-0.5, 0.25};
  autoware_utils_geometry::Point2d right_front{0.5, 0.25};
  autoware_utils_geometry::Point2d right_rear{0.5, -0.25};
  autoware_utils_geometry::Point2d left_rear{-0.5, -0.25};
  auto traj = build_parabolic_trajectory(11, 0.5);
  // LinearRing2d
  {
    autoware_utils_geometry::LinearRing2d base_ring{left_front, right_front, right_rear, left_rear};

    const auto footprints = autoware::experimental::trajectory::build_path_footprints(
      traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 1.0, base_ring);

    EXPECT_EQ(
      footprints.size(),
      static_cast<int>(traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1] + 1));
  }

  // Polygon2d
  {
    autoware_utils_geometry::Polygon2d base_polygon;
    base_polygon.outer().push_back(left_front);
    base_polygon.outer().push_back(right_front);
    base_polygon.outer().push_back(right_rear);
    base_polygon.outer().push_back(left_rear);

    const auto footprints = autoware::experimental::trajectory::build_path_footprints(
      traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], 1.0,
      base_polygon);

    EXPECT_EQ(
      footprints.size(),
      static_cast<int>(traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1] + 1));
  }
}

// Test 3: count numbers of footprint trace with designated interval
TEST(trajectoryFootprint, FootprintTraceAdjustInterval)
{
  autoware_utils_geometry::Point2d left_front{-0.5, 0.25};
  autoware_utils_geometry::Point2d right_front{0.5, 0.25};
  autoware_utils_geometry::Point2d right_rear{0.5, -0.25};
  autoware_utils_geometry::Point2d left_rear{-0.5, -0.25};
  auto traj = build_parabolic_trajectory(11, 0.5);
  const double interval = 2.0;
  // LinearRing2d
  {
    autoware_utils_geometry::LinearRing2d base_ring{left_front, right_front, right_rear, left_rear};

    const auto footprints = autoware::experimental::trajectory::build_path_footprints(
      traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], interval,
      base_ring);

    EXPECT_EQ(
      footprints.size(),
      static_cast<int>(
        traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1] / interval + 1));
  }

  // Polygon2d
  {
    autoware_utils_geometry::Polygon2d base_polygon;
    base_polygon.outer().push_back(left_front);
    base_polygon.outer().push_back(right_front);
    base_polygon.outer().push_back(right_rear);
    base_polygon.outer().push_back(left_rear);

    const auto footprints = autoware::experimental::trajectory::build_path_footprints(
      traj, 0, traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1], interval,
      base_polygon);

    EXPECT_EQ(
      footprints.size(),
      static_cast<int>(
        traj.get_underlying_bases()[traj.get_underlying_bases().size() - 1] / interval + 1));
  }
}

}  // namespace

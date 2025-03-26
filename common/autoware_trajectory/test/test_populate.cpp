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

#include "autoware/trajectory/utils/populate.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_utils_geometry::create_quaternion_from_yaw;
using autoware_utils_geometry::get_rpy;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

TEST(populate3, populate3_succeed)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(3.0).y(3.0).z(1.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points3_result = autoware::trajectory::populate3(points);
  EXPECT_TRUE(points3_result);
  const auto & points3 = points3_result.value();
  EXPECT_EQ(points3.size(), 3);

  // first and last point is same
  const auto &p1 = points3.at(0), p2 = points3.at(1), p3 = points3.at(2);  // NOLINT
  {
    EXPECT_EQ(p1.point.pose.position.x, 1.0);
    EXPECT_EQ(p1.point.pose.position.y, 1.0);
    EXPECT_EQ(p1.point.pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(get_rpy(p1.point.pose.orientation).z, 0.0);
    EXPECT_EQ(p1.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p1.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p1.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p1.lane_ids.size() == 1 && p1.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p3.point.pose.position.x, 3.0);
    EXPECT_EQ(p3.point.pose.position.y, 3.0);
    EXPECT_EQ(p3.point.pose.position.z, 1.0);
    EXPECT_FLOAT_EQ(get_rpy(p3.point.pose.orientation).z, M_PI / 2.0);
    EXPECT_EQ(p3.point.longitudinal_velocity_mps, 20.0);
    EXPECT_EQ(p3.point.lateral_velocity_mps, 1.0);
    EXPECT_EQ(p3.point.heading_rate_rps, 1.0);
    EXPECT_EQ(p3.lane_ids.size() == 1 && p3.lane_ids.at(0) == 2, true);
  }
  // mid point
  {
    EXPECT_EQ(p2.point.pose.position.x, 2.0);
    EXPECT_EQ(p2.point.pose.position.y, 2.0);
    EXPECT_EQ(p2.point.pose.position.z, 0.5);
    EXPECT_FLOAT_EQ(get_rpy(p2.point.pose.orientation).z, M_PI / 4.0);
    EXPECT_EQ(p2.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p2.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p2.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p2.lane_ids.size() == 1 && p2.lane_ids.at(0) == 1, true);
  }
}

TEST(populate3, populate3_succeed_skip)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(2.0).y(2.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(3.0).y(3.0).z(1.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points_result = autoware::trajectory::populate3(points);
  EXPECT_TRUE(points_result);
  EXPECT_EQ(points_result.value().size(), 3);
}

TEST(populate3, populate3_fail)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  const auto points_result = autoware::trajectory::populate3(points);
  EXPECT_TRUE(!points_result);
}

TEST(populate4, populate4_succeed_from_2_to_4_1)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(4.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::trajectory::populate4(points);
  EXPECT_TRUE(points4_result);
  const auto & points4 = points4_result.value();
  EXPECT_EQ(points4.size(), 4);
  // NOLINTBEGIN
  const auto &p1 = points4.at(0), p2 = points4.at(1), p3 = points4.at(2), p4 = points4.at(3);
  // NOLINTEND
  {
    EXPECT_EQ(p1.point.pose.position.x, 0.0);
    EXPECT_EQ(p1.point.pose.position.y, 0.0);
    EXPECT_EQ(p1.point.pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(get_rpy(p1.point.pose.orientation).z, 0.0);
    EXPECT_EQ(p1.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p1.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p1.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p1.lane_ids.size() == 1 && p1.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p2.point.pose.position.x, 1.0);
    EXPECT_EQ(p2.point.pose.position.y, 1.0);
    EXPECT_EQ(p2.point.pose.position.z, 1.0);
    EXPECT_FLOAT_EQ(get_rpy(p2.point.pose.orientation).z, M_PI / 8.0);
    EXPECT_EQ(p2.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p2.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p2.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p2.lane_ids.size() == 1 && p2.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p3.point.pose.position.x, 2.0);
    EXPECT_EQ(p3.point.pose.position.y, 2.0);
    EXPECT_EQ(p3.point.pose.position.z, 2.0);
    EXPECT_FLOAT_EQ(get_rpy(p3.point.pose.orientation).z, M_PI / 4.0);
    EXPECT_EQ(p3.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p3.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p3.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p3.lane_ids.size() == 1 && p3.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p4.point.pose.position.x, 4.0);
    EXPECT_EQ(p4.point.pose.position.y, 4.0);
    EXPECT_EQ(p4.point.pose.position.z, 4.0);
    EXPECT_FLOAT_EQ(get_rpy(p4.point.pose.orientation).z, M_PI / 2.0);
    EXPECT_EQ(p4.point.longitudinal_velocity_mps, 20.0);
    EXPECT_EQ(p4.point.lateral_velocity_mps, 1.0);
    EXPECT_EQ(p4.point.heading_rate_rps, 1.0);
    EXPECT_EQ(p4.lane_ids.size() == 1 && p4.lane_ids.at(0) == 2, true);
  }
}

TEST(populate4, populate4_succeed_from_3_to_4_1)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(1.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(4.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::trajectory::populate4(points);
  EXPECT_TRUE(points4_result);
  const auto & points4 = points4_result.value();
  EXPECT_EQ(points4.size(), 4);
  // NOLINTBEGIN
  const auto &p1 = points4.at(0), p2 = points4.at(1), p3 = points4.at(2), p4 = points4.at(3);
  // NOLINTEND
  {
    EXPECT_EQ(p1.point.pose.position.x, 0.0);
    EXPECT_EQ(p1.point.pose.position.y, 0.0);
    EXPECT_EQ(p1.point.pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(get_rpy(p1.point.pose.orientation).z, 0.0);
    EXPECT_EQ(p1.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p1.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p1.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p1.lane_ids.size() == 1 && p1.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p2.point.pose.position.x, 1.0);
    EXPECT_EQ(p2.point.pose.position.y, 1.0);
    EXPECT_EQ(p2.point.pose.position.z, 1.0);
    EXPECT_FLOAT_EQ(get_rpy(p2.point.pose.orientation).z, 0.0);
    EXPECT_EQ(p2.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p2.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p2.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p2.lane_ids.size() == 1 && p2.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p3.point.pose.position.x, 2.5);
    EXPECT_EQ(p3.point.pose.position.y, 2.5);
    EXPECT_EQ(p3.point.pose.position.z, 2.5);
    EXPECT_FLOAT_EQ(get_rpy(p3.point.pose.orientation).z, M_PI / 4.0);
    EXPECT_EQ(p3.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p3.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p3.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p3.lane_ids.size() == 1 && p3.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p4.point.pose.position.x, 4.0);
    EXPECT_EQ(p4.point.pose.position.y, 4.0);
    EXPECT_EQ(p4.point.pose.position.z, 4.0);
    EXPECT_FLOAT_EQ(get_rpy(p4.point.pose.orientation).z, M_PI / 2.0);
    EXPECT_EQ(p4.point.longitudinal_velocity_mps, 20.0);
    EXPECT_EQ(p4.point.lateral_velocity_mps, 1.0);
    EXPECT_EQ(p4.point.heading_rate_rps, 1.0);
    EXPECT_EQ(p4.lane_ids.size() == 1 && p4.lane_ids.at(0) == 2, true);
  }
}

TEST(populate4, populate4_succeed_from_3_to_4_2)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(3.0).y(3.0).z(3.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(4.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::trajectory::populate4(points);
  EXPECT_TRUE(points4_result);
  const auto & points4 = points4_result.value();
  EXPECT_EQ(points4.size(), 4);
  // NOLINTBEGIN
  const auto &p1 = points4.at(0), p2 = points4.at(1), p3 = points4.at(2), p4 = points4.at(3);
  // NOLINTEND
  {
    EXPECT_EQ(p1.point.pose.position.x, 0.0);
    EXPECT_EQ(p1.point.pose.position.y, 0.0);
    EXPECT_EQ(p1.point.pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(get_rpy(p1.point.pose.orientation).z, 0.0);
    EXPECT_EQ(p1.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p1.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p1.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p1.lane_ids.size() == 1 && p1.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p2.point.pose.position.x, 1.5);
    EXPECT_EQ(p2.point.pose.position.y, 1.5);
    EXPECT_EQ(p2.point.pose.position.z, 1.5);
    EXPECT_FLOAT_EQ(get_rpy(p2.point.pose.orientation).z, M_PI / 4.0);
    EXPECT_EQ(p2.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p2.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p2.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p2.lane_ids.size() == 1 && p2.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p3.point.pose.position.x, 3.0);
    EXPECT_EQ(p3.point.pose.position.y, 3.0);
    EXPECT_EQ(p3.point.pose.position.z, 3.0);
    EXPECT_FLOAT_EQ(get_rpy(p3.point.pose.orientation).z, M_PI / 2.0);
    EXPECT_EQ(p3.point.longitudinal_velocity_mps, 10.0);
    EXPECT_EQ(p3.point.lateral_velocity_mps, 0.5);
    EXPECT_EQ(p3.point.heading_rate_rps, 0.5);
    EXPECT_EQ(p3.lane_ids.size() == 1 && p3.lane_ids.at(0) == 1, true);
  }
  {
    EXPECT_EQ(p4.point.pose.position.x, 4.0);
    EXPECT_EQ(p4.point.pose.position.y, 4.0);
    EXPECT_EQ(p4.point.pose.position.z, 4.0);
    EXPECT_FLOAT_EQ(get_rpy(p4.point.pose.orientation).z, M_PI / 2.0);
    EXPECT_EQ(p4.point.longitudinal_velocity_mps, 20.0);
    EXPECT_EQ(p4.point.lateral_velocity_mps, 1.0);
    EXPECT_EQ(p4.point.heading_rate_rps, 1.0);
    EXPECT_EQ(p4.lane_ids.size() == 1 && p4.lane_ids.at(0) == 2, true);
  }
}

TEST(populate4, populate4_succeed_skip)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(2.0).y(2.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(3.0).y(3.0).z(1.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(2.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points_result = autoware::trajectory::populate4(points);
  EXPECT_TRUE(points_result);
  EXPECT_EQ(points_result.value().size(), 4);
}

TEST(populate4, populate4_fail)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(1.0).y(1.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  const auto points_result = autoware::trajectory::populate4(points);
  EXPECT_TRUE(!points_result);
}

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

#include "test_utils.hpp"

#include <vector>

namespace test_utils
{

geometry_msgs::msg::Pose create_test_pose(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> create_test_trajectory()
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  // Generate trajectory data for testing
  for (double x = 0.0; x <= 300.0; x += 1.0) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    traj_points.push_back(point);
  }
  return traj_points;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> create_test_decimated_trajectory()
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> decimated_traj_points;
  // Generate decimated trajectory data for testing
  for (double x = 0.0; x <= 300.0; x += 2.0) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    decimated_traj_points.push_back(point);
  }
  return decimated_traj_points;
}

}  // namespace test_utils

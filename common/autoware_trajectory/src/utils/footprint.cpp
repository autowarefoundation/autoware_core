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

#include "autoware/trajectory/utils/footprint.hpp"

#include <vector>

namespace autoware::experimental::trajectory
{
// ============================= Extern Template ===========================================

template autoware_utils_geometry::Polygon2d
build_path_polygon<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory, const double start_s,
  const double end_s, const double interval, const double width);

template autoware_utils_geometry::Polygon2d
build_path_polygon<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const double start_s, const double end_s, const double interval, const double width);

template autoware_utils_geometry::Polygon2d build_path_polygon<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const double start_s, const double end_s,
  const double interval, const double width);

template autoware_utils_geometry::Polygon2d
build_path_polygon<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double start_s,
  const double end_s, const double interval, const double width);

// ===================== build_path_footprints (LinearRing2d) ==============

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory, const double start_s,
  const double end_s, const double interval,
  const autoware_utils_geometry::LinearRing2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const double start_s, const double end_s, const double interval,
  const autoware_utils_geometry::LinearRing2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const double start_s, const double end_s,
  const double interval, const autoware_utils_geometry::LinearRing2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double start_s,
  const double end_s, const double interval,
  const autoware_utils_geometry::LinearRing2d & base_footprint);

// ====================== build_path_footprints (Polygon2d) ================

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory, const double start_s,
  const double end_s, const double interval,
  const autoware_utils_geometry::Polygon2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const double start_s, const double end_s, const double interval,
  const autoware_utils_geometry::Polygon2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const double start_s, const double end_s,
  const double interval, const autoware_utils_geometry::Polygon2d & base_footprint);

template std::vector<autoware_utils_geometry::Polygon2d>
build_path_footprints<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double start_s,
  const double end_s, const double interval,
  const autoware_utils_geometry::Polygon2d & base_footprint);

}  // namespace autoware::experimental::trajectory

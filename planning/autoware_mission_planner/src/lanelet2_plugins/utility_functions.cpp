// Copyright 2019-2024 Autoware Foundation
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

#include "utility_functions.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <limits>
#include <vector>

namespace
{

lanelet::Point3d remove_const(const lanelet::ConstPoint3d & point)
{
  return lanelet::Point3d{std::const_pointer_cast<lanelet::PointData>(point.constData())};
}

lanelet::Lanelet remove_const(const lanelet::ConstLanelet const_lanelet)
{
  lanelet::ConstLineString3d left_cls = const_lanelet.leftBound();
  lanelet::ConstLineString3d right_cls = const_lanelet.rightBound();

  lanelet::LineString3d left_ls;
  lanelet::LineString3d right_ls;

  std::for_each(left_cls.begin(), left_cls.end(), [&](const auto & point) {
    left_ls.push_back(remove_const(point));
  });

  std::for_each(right_cls.begin(), right_cls.end(), [&](const auto & point) {
    right_ls.push_back(remove_const(point));
  });

  lanelet::Lanelet ll(lanelet::InvalId, left_ls, right_ls);
  return ll;
}
}  // namespace

namespace autoware::mission_planner::lanelet2
{
autoware_utils_geometry::Polygon2d convert_linear_ring_to_polygon(
  autoware_utils_geometry::LinearRing2d footprint)
{
  autoware_utils_geometry::Polygon2d footprint_polygon;
  boost::geometry::append(footprint_polygon.outer(), footprint[0]);
  boost::geometry::append(footprint_polygon.outer(), footprint[1]);
  boost::geometry::append(footprint_polygon.outer(), footprint[2]);
  boost::geometry::append(footprint_polygon.outer(), footprint[3]);
  boost::geometry::append(footprint_polygon.outer(), footprint[4]);
  boost::geometry::append(footprint_polygon.outer(), footprint[5]);
  boost::geometry::correct(footprint_polygon);
  return footprint_polygon;
}

void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet & lanelet)
{
  std::vector<geometry_msgs::msg::Point> centerline_points;
  for (const auto & point : lanelet.centerline()) {
    geometry_msgs::msg::Point center_point;
    center_point.x = point.basicPoint().x();
    center_point.y = point.basicPoint().y();
    center_point.z = point.basicPoint().z();
    centerline_points.push_back(center_point);
  }
  return centerline_points;
}

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw)
{
  // calculate new orientation of goal
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(lane_yaw);

  return pose;
}

bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool is_in_parking_space(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

bool is_in_parking_lot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

double project_goal_to_map(
  const lanelet::ConstLanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point)
{
  const lanelet::ConstLineString3d center_line =
    lanelet::utils::generateFineCenterline(lanelet_component);
  lanelet::BasicPoint3d project = lanelet::geometry::project(center_line, goal_point.basicPoint());
  return project.z();
}

geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets & road_lanelets, const geometry_msgs::msg::Pose & point,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info)
{
  auto opt = autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint(
    road_lanelets, point, 0.0);
  if (!opt.has_value()) {
    // point is not on any lanelet.
    return point;
  }

  lanelet::Lanelet closest_lanelet = remove_const(*opt);

  const auto refined_center_line = lanelet::utils::generateFineCenterline(closest_lanelet, 1.0);
  closest_lanelet.setCenterline(refined_center_line);

  const double lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
    closest_lanelet, autoware::experimental::lanelet2_utils::from_ros(point.position).basicPoint());

  const auto nearest_idx = autoware::motion_utils::findNearestIndex(
    convertCenterlineToPoints(closest_lanelet), point.position);
  const auto nearest_point = closest_lanelet.centerline()[nearest_idx];

  // shift nearest point on its local y axis so that vehicle's right and left edges
  // would have approx the same clearance from road border
  const auto shift_length = (vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const auto delta_x = -shift_length * std::sin(lane_yaw);
  const auto delta_y = shift_length * std::cos(lane_yaw);

  lanelet::BasicPoint3d refined_point(
    nearest_point.x() + delta_x, nearest_point.y() + delta_y, nearest_point.z());

  return convertBasicPoint3dToPose(refined_point, lane_yaw);
}

}  // namespace autoware::mission_planner::lanelet2

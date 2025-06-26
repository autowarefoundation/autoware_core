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

#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_position;
using autoware_utils_visualization::create_marker_scale;

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  const auto & points = polygon.points;
  const size_t N = points.size();

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    marker.points.reserve(2 * N);
    for (size_t i = 0; i < N; ++i) {
      const auto & cur = points[i];
      const auto & nxt = points[(i + 1) % N];
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x;
      p1.y = cur.y;
      p1.z = cur.z;
      p2.x = nxt.x;
      p2.y = nxt.y;
      p2.z = nxt.z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    marker.points.reserve(N + 1);
    for (const auto & p : points) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < area_polygons.size(); ++i) {
    const auto marker = create_autoware_geometry_marker(
      area_polygons[i], stamp, ns, id, marker_type, scale, color, z);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & multi_polygon, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  for (const auto & polygon : multi_polygon) {
    marker.points.push_back(trajectory[trajectory_index].pose.position);
    const auto centroid =
      boost::geometry::return_centroid<autoware_utils_geometry::Point2d>(polygon);
    marker.points.push_back(geometry_msgs::msg::Point().set__x(centroid.x()).set__y(centroid.y()));
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Point & stop_obstacle_point, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  marker.pose.position = stop_obstacle_point;
  marker.pose.position.z += 2.0;

  marker.text = "!";
  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::LinearRing2d & ring, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  for (size_t i = 0; i < ring.size(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = ring[i][0];
    pt.y = ring[i][1];
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  marker_array.markers.push_back(marker);

  return marker_array;
}

visualization_msgs::msg::Marker create_autoware_geometry_marker(
  const autoware_utils_geometry::Polygon2d & polygon, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::Marker marker =
    create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    for (size_t i = 0; i < polygon.outer().size(); ++i) {
      const auto & cur = polygon.outer().at(i);
      const auto & nxt = polygon.outer().at((i + 1) % polygon.outer().size());
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x();
      p1.y = cur.y();
      p1.z = z;
      p2.x = nxt.x();
      p2.y = nxt.y();
      p2.z = z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    marker.pose.orientation = autoware_utils_visualization::create_marker_orientation(0, 0, 0, 1.0);
    for (const auto & p : polygon.outer()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = z;
      marker.points.push_back(pt);
    }
  }
  return marker;
}

visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & stamp,
  const std::string & ns, const int32_t id, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;

  auto marker = create_default_marker(
    "map", stamp, ns, 0, visualization_msgs::msg::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    color);
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects.at(i);
    marker.id = static_cast<int>((id << (sizeof(int32_t) * 8 / 2)) + static_cast<int32_t>(i));
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

// helper
static void create_vehicle_footprint_marker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const double & base_to_right, const double & base_to_left, const double & base_to_front,
  const double & base_to_rear)
{
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, base_to_front, base_to_left, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, base_to_front, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, -base_to_rear, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, -base_to_rear, base_to_left, 0.0).position);
  marker.points.push_back(marker.points.front());
}

static std::vector<double> calc_path_arc_length_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::vector<double> out;
  if (path.points.empty()) return out;

  out.reserve(path.points.size());
  double sum = 0.0;
  out.push_back(sum);

  for (size_t i = 1; i < path.points.size(); ++i) {
    sum += autoware_utils_geometry::calc_distance2d(
      path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

visualization_msgs::msg::MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t id)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.99, 0.99, 0.2, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
  const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    marker.id = i;
    marker.points.clear();

    const auto & traj_point = mpt_traj.at(i);
    create_vehicle_footprint_marker(
      marker, traj_point.pose, base_to_right, base_to_left, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const std_msgs::msg::ColorRGBA & color)
{
  if (predicted_path.path.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  const auto & path = predicted_path.path;

  visualization_msgs::msg::Marker marker = create_default_marker(
    "map", current_time, ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color);
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  visualization_msgs::msg::MarkerArray marker_array;
  const double half_width = -vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  for (size_t i = 0; i < path.size(); ++i) {
    marker.id = i + id;
    marker.points.clear();

    const auto & predicted_path_pose = path.at(i);
    create_vehicle_footprint_marker(
      marker, predicted_path_pose, half_width, half_width, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_path_with_lane_id_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int32_t id, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text)
{
  auto uid = id << (sizeof(int32_t) * 8 / 2);
  int32_t idx = 0;
  int32_t i = 0;
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker = create_default_marker(
    "map", now, ns, static_cast<int32_t>(uid), visualization_msgs::msg::Marker::ARROW, scale,
    color);

  for (const auto & p : path.points) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.pose = p.point.pose;

    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), id) == p.lane_ids.end() && !with_text) {
      marker.color = create_marker_color(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
    if (i % 10 == 0 && with_text) {
      const auto arclength = calc_path_arc_length_array(path);
      visualization_msgs::msg::Marker marker_text = create_default_marker(
        "map", now, ns, 0L, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        create_marker_scale(0.2, 0.1, 0.3), create_marker_color(1, 1, 1, 0.999));
      marker_text.id = uid + i++;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }
  return msg;
}

visualization_msgs::msg::MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const geometry_msgs::msg::Vector3 scale, const double z,
  const bool planning)
{
  if (lanelets.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  visualization_msgs::msg::MarkerArray marker_array;

  if (planning) {
    if (ns.empty()) {
      return lanelet::visualization::laneletsBoundaryAsMarkerArray(lanelets, color, false);
    } else {
      return lanelet::visualization::laneletsAsTriangleMarkerArray(ns, lanelets, color);
    }
  }

  auto marker = create_default_marker(
    "map", rclcpp::Time(0), ns, 0, visualization_msgs::msg::Marker::LINE_LIST, scale, color);

  for (const auto & ll : lanelets) {
    marker.points.clear();
    for (const auto & p : ll.polygon2d().basicPolygon()) {
      marker.points.push_back(create_marker_position(p.x(), p.y(), z + 0.5));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
    ++marker.id;
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::CompoundPolygon3d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker(
    "map", stamp, ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.0, 0.0), color);
  for (const auto & p : polygon) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    marker.points.push_back(pt);
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

}  // namespace autoware::experimental::marker_utils

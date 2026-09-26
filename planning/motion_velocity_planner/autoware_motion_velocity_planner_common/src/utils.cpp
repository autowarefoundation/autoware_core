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

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_velocity_planner_common/planner_data.hpp"
#include "autoware/motion_velocity_planner_common/velocity_planning_result.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
namespace
{
// Below this curvature the arc formula divides by an almost-zero radius, and the road is straight
// enough that the difference is irrelevant: 1e-4 corresponds to a 10 km radius.
constexpr double min_curvature_for_arc = 1e-4;

// Index of the point roughly `distance` metres back from the end of `points`, clamped to the
// start. Used to spread the curvature samples over a fixed baseline rather than over whatever
// spacing the trajectory happens to have.
size_t index_before_end(const std::vector<TrajectoryPoint> & points, const double distance)
{
  double travelled = 0.0;
  for (size_t i = points.size() - 1; i > 0; --i) {
    travelled += autoware_utils_geometry::calc_distance2d(
      points.at(i).pose.position, points.at(i - 1).pose.position);
    if (travelled >= distance) {
      return i - 1;
    }
  }
  return 0;
}

// Curvature of the road at the end of `points`, from three points spread over `baseline_length`.
// Three points on a circle give its curvature exactly, so a clean arc needs no more than that; the
// baseline is what keeps a densely sampled trajectory from turning rounding noise into curvature.
// Returns zero (straight) when there are too few points, or when they are too close together or
// too collinear to define a circle.
double estimate_goal_curvature(
  const std::vector<TrajectoryPoint> & points, const double baseline_length)
{
  if (points.size() < 3) {
    return 0.0;
  }
  size_t middle = index_before_end(points, baseline_length);
  size_t first = index_before_end(points, 2.0 * baseline_length);
  // Fall back to the last three points when the trajectory is shorter than the baseline.
  if (first == middle || middle == points.size() - 1) {
    first = points.size() - 3;
    middle = points.size() - 2;
  }
  try {
    return autoware_utils_geometry::calc_curvature(
      points.at(first).pose.position, points.at(middle).pose.position, points.back().pose.position);
  } catch (const std::runtime_error &) {
    return 0.0;
  }
}

TrajectoryPoint extend_trajectory_point(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward,
  const double curvature)
{
  const double signed_distance = extend_distance * (is_driving_forward ? 1.0 : -1.0);

  // Follow the road rather than the tangent. In the goal frame, travelling signed_distance along a
  // circle of the given curvature lands at (sin(t) / k, (1 - cos(t)) / k) having turned by
  // t = k * signed_distance. As k tends to zero this tends to (signed_distance, 0) with no turn,
  // which is the straight extension used below the threshold.
  double x = signed_distance;
  double y = 0.0;
  double yaw = 0.0;
  if (std::abs(curvature) >= min_curvature_for_arc) {
    const double turn = curvature * signed_distance;
    x = std::sin(turn) / curvature;
    y = (1.0 - std::cos(turn)) / curvature;
    yaw = turn;
  }

  TrajectoryPoint extended_trajectory_point;
  extended_trajectory_point.pose =
    autoware_utils_geometry::calc_offset_pose(goal_point.pose, x, y, 0.0, yaw);
  extended_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extended_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extended_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extended_trajectory_point;
}

}  // namespace

std::vector<TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length, const std::optional<double> curvature)
{
  auto output_points = input_points;
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  // A value to prevent division-by-zero in curvature math while ensuring adequate precision.
  constexpr double min_step_length = 0.1;
  if (extend_distance < min_step_length) {
    return output_points;
  }

  // Guard against dereferencing back() on an empty trajectory: there is no goal point to extend
  // from, so return the (empty) input unchanged.
  if (input_points.empty()) {
    return output_points;
  }

  const auto goal_point = input_points.back();
  const double goal_curvature =
    curvature.value_or(estimate_goal_curvature(input_points, step_length));
  // Stop one epsilon short of extend_distance so that a step landing exactly on it does not add a
  // duplicate of the final point below.
  constexpr double duplicate_point_epsilon = 1e-6;
  for (double extend_sum = step_length; extend_sum < extend_distance - duplicate_point_epsilon;
       extend_sum += step_length) {
    output_points.push_back(
      extend_trajectory_point(extend_sum, goal_point, is_driving_forward, goal_curvature));
  }
  output_points.push_back(
    extend_trajectory_point(extend_distance, goal_point, is_driving_forward, goal_curvature));

  return output_points;
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj_msg = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj_msg = autoware::motion_utils::resampleTrajectory(traj_msg, interval);
  auto resampled_traj = autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj_msg);
  const bool is_driving_forward =
    autoware_utils_geometry::is_driving_forward(traj_points.at(0), traj_points.at(1));
  autoware::motion_utils::insertOrientationAsSpline(resampled_traj, is_driving_forward);
  return resampled_traj;
}

std::vector<TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length, const double goal_extended_trajectory_length)
{
  // trim trajectory points from ego pose
  const size_t traj_ego_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const auto traj_points_from_ego = std::vector<TrajectoryPoint>(
    traj_points.begin() + static_cast<std::ptrdiff_t>(traj_ego_seg_idx), traj_points.end());

  // decimate trajectory
  const auto decimated_traj_points_from_ego =
    resample_trajectory_points(traj_points_from_ego, decimate_trajectory_step_length);

  // extend trajectory. Measure the curvature on the untrimmed trajectory: close to the goal the
  // trimmed and decimated one is only a couple of points long, which is exactly when the
  // extension past the goal decides whether an obstacle beyond it is seen.
  const auto extended_traj_points_from_ego = get_extended_trajectory_points(
    decimated_traj_points_from_ego, goal_extended_trajectory_length,
    decimate_trajectory_step_length,
    estimate_goal_curvature(traj_points, decimate_trajectory_step_length));
  if (extended_traj_points_from_ego.size() < 2) {
    return traj_points;
  }
  return extended_traj_points_from_ego;
}
geometry_msgs::msg::Point to_geometry_point(const pcl::PointXYZ & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x;
  geom_point.y = point.y;
  geom_point.z = point.z;
  return geom_point;
}

geometry_msgs::msg::Point to_geometry_point(const autoware_utils_geometry::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  const size_t obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle_pos);
  const auto ego_to_obstacle_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);
  if (ego_to_obstacle_distance < 0.0) return std::nullopt;
  return ego_to_obstacle_distance;
}

std::vector<uint8_t> get_target_object_type(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, uint8_t> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN},
    {"animal", ObjectClassification::ANIMAL},   {"hazard", ObjectClassification::HAZARD}};

  std::vector<uint8_t> types;
  for (const auto & type : types_map) {
    const auto param_name = param_prefix + type.first;
    const bool is_target_object = node.has_parameter(param_name)
                                    ? node.get_parameter(param_name).as_bool()
                                    : node.declare_parameter<bool>(param_name, false);
    if (is_target_object) {
      types.push_back(type.second);
    }
  }
  return types;
}

double calc_object_possible_max_dist_from_center(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in motion_velocity_planner_common.");
}
visualization_msgs::msg::Marker get_object_marker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();

  auto marker = autoware_utils_visualization::create_default_marker(
    "map", current_time, ns, static_cast<int32_t>(idx), visualization_msgs::msg::Marker::SPHERE,
    autoware_utils_visualization::create_marker_scale(2.0, 2.0, 2.0),
    autoware_utils_visualization::create_marker_color(
      static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8));

  marker.pose = obj_pose;

  return marker;
}

double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info)
{
  const double object_possible_max_dist =
    calc_object_possible_max_dist_from_center(object->predicted_object.shape);
  // The minimum lateral distance to the trajectory polygon is estimated by assuming that the
  // ego-vehicle's front right or left corner is the furthest from the trajectory, in the very worst
  // case
  const double ego_possible_max_dist =
    std::hypot(vehicle_info.max_longitudinal_offset_m, vehicle_info.vehicle_width_m / 2.0);
  const double possible_min_dist_to_traj_poly =
    std::abs(object->get_dist_to_traj_lateral(traj_points)) - ego_possible_max_dist -
    object_possible_max_dist;
  return possible_min_dist_to_traj_poly;
}

double get_dist_to_traj_poly(
  const geometry_msgs::msg::Point & point,
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys)
{
  const auto point_2d = autoware_utils_geometry::Point2d(point.x, point.y);

  double dist_to_traj_poly = std::numeric_limits<double>::infinity();

  for (const auto & decimated_traj_poly : decimated_traj_polys) {
    const double current_dist_to_traj_poly =
      boost::geometry::distance(decimated_traj_poly, point_2d);

    dist_to_traj_poly = std::min(dist_to_traj_poly, current_dist_to_traj_poly);
  }

  return dist_to_traj_poly;
}

double calc_dist_to_traj_poly(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys)
{
  const auto obj_poly = autoware_utils_geometry::to_polygon2d(
    predicted_object.kinematics.initial_pose_with_covariance.pose, predicted_object.shape);
  double dist_to_traj_poly = std::numeric_limits<double>::max();
  for (const auto & traj_poly : decimated_traj_polys) {
    const double current_dist_to_traj_poly = boost::geometry::distance(traj_poly, obj_poly);
    dist_to_traj_poly = std::min(dist_to_traj_poly, current_dist_to_traj_poly);
  }
  return dist_to_traj_poly;
}

void insert_stop(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & stop_point, const rclcpp::Logger & logger)
{
  // Prevent sudden yaw angle changes by using a larger overlap threshold
  // when inserting stop points that are very close to existing points
  const double overlap_threshold = 5e-2;
  const auto seg_idx = autoware::motion_utils::findNearestSegmentIndex(trajectory, stop_point);
  const auto insert_idx =
    autoware::motion_utils::insertTargetPoint(seg_idx, stop_point, trajectory, overlap_threshold);
  if (insert_idx) {
    for (auto idx = *insert_idx; idx < trajectory.size(); ++idx)
      trajectory[idx].longitudinal_velocity_mps = 0.0;
  } else {
    RCLCPP_WARN(logger, "Failed to insert stop point");
  }
}

void insert_slowdown(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware::motion_velocity_planner::SlowdownInterval & slowdown_interval,
  const rclcpp::Logger & logger)
{
  const auto from_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory, slowdown_interval.from);
  const auto from_insert_idx =
    autoware::motion_utils::insertTargetPoint(from_seg_idx, slowdown_interval.from, trajectory);
  const auto to_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory, slowdown_interval.to);
  const auto to_insert_idx =
    autoware::motion_utils::insertTargetPoint(to_seg_idx, slowdown_interval.to, trajectory);
  if (from_insert_idx && to_insert_idx) {
    for (auto idx = *from_insert_idx; idx <= *to_insert_idx; ++idx) {
      trajectory[idx].longitudinal_velocity_mps =
        std::min(  // prevent the node from increasing the velocity
          trajectory[idx].longitudinal_velocity_mps,
          static_cast<float>(slowdown_interval.velocity));
    }
  } else {
    RCLCPP_WARN(logger, "Failed to insert slowdown point");
  }
}
void apply_planning_result(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware::motion_velocity_planner::VelocityPlanningResult & planning_result,
  const rclcpp::Logger & logger)
{
  for (const auto & stop_point : planning_result.stop_points)
    insert_stop(trajectory, stop_point, logger);
  for (const auto & slowdown_interval : planning_result.slowdown_intervals)
    insert_slowdown(trajectory, slowdown_interval, logger);
}
}  // namespace autoware::motion_velocity_planner::utils

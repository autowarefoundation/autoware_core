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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{

namespace
{

size_t compute_num_segments(const lanelet::ConstLanelet & lanelet, const double resolution)
{
  // Get length of longer border
  const double left_length = static_cast<double>(lanelet::geometry::length(lanelet.leftBound()));
  const double right_length = static_cast<double>(lanelet::geometry::length(lanelet.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const size_t num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);
  return num_segments;
}

lanelet::BasicPoints3d resample_points(
  const lanelet::BasicLineString3d & line_string, const size_t num_segments)
{
  // Note: this function works well with num_segments >= num_segment of line string
  // less than this, it might cause loss in information (such as corner curve)
  if (line_string.size() < 2 || num_segments == 0) {
    return {};
  }

  // Compute accumulated arc length
  std::vector<double> accumulated_lengths{0.0};
  accumulated_lengths.reserve(line_string.size());
  for (size_t i = 1; i < line_string.size(); ++i) {
    const double distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    accumulated_lengths.push_back(accumulated_lengths.back() + distance);
  }

  const double total_length = accumulated_lengths.back();

  lanelet::BasicPoints3d resampled_points;
  for (size_t i = 0; i <= num_segments; ++i) {
    const double target_length = total_length * static_cast<double>(i) / num_segments;

    // Find two nearest points
    // (accumulated_lengths[idx-1] < target_length <= accumulated_lengths[idx])
    auto it =
      std::lower_bound(accumulated_lengths.begin(), accumulated_lengths.end(), target_length);
    size_t idx = std::distance(accumulated_lengths.begin(), it);

    if (idx == 0) {
      resampled_points.push_back(line_string.front());
      continue;
    } else if (idx >= accumulated_lengths.size()) {
      resampled_points.push_back(line_string.back());
      break;
    }

    const double back_length = accumulated_lengths[idx - 1];
    const double front_length = accumulated_lengths[idx];
    const double ratio = (target_length - back_length) / (front_length - back_length);

    const lanelet::BasicPoint3d back_point = line_string[idx - 1];
    const lanelet::BasicPoint3d front_point = line_string[idx];

    const lanelet::BasicPoint3d direction_vector = front_point - back_point;
    const lanelet::BasicPoint3d target_point = back_point + direction_vector * ratio;

    resampled_points.push_back(target_point);
  }
  return resampled_points;
}
}  // namespace

lanelet::ConstPoint3d extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance)
{
  const double segment_length = lanelet::geometry::distance3d(first, second);
  if (segment_length == 0.0) {
    return first;
  }

  return lanelet::ConstPoint3d{
    lanelet::InvalId,
    second.basicPoint() + (second.basicPoint() - first.basicPoint()).normalized() * distance};
}

std::optional<lanelet::ConstPoint3d> interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance)
{
  const double segment_length = lanelet::geometry::distance3d(first, second);

  if (segment_length == 0.0 || distance < 0.0 || distance > segment_length) {
    return std::nullopt;
  }

  return lanelet::ConstPoint3d{
    lanelet::InvalId,
    first.basicPoint() + (second.basicPoint() - first.basicPoint()).normalized() * distance};
}

template <typename Line>
std::optional<lanelet::ConstPoint3d> interpolate_linestring(
  const Line & linestring, double distance)
{
  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = lanelet::geometry::length(linestring);
  if (distance < 0.0 || distance > total_length) {
    return std::nullopt;
  }

  double accumulated = 0.0;
  for (std::size_t i = 0; i + 1 < linestring.size(); ++i) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double seg_len = lanelet::geometry::distance3d(p1, p2);
    if (accumulated + seg_len >= distance) {
      return interpolate_point(p1, p2, distance - accumulated);
    }
    accumulated += seg_len;
  }

  return std::nullopt;
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet(
  const lanelet::ConstLanelet & lanelet, const double distance)
{
  const auto linestring = lanelet.centerline();
  return interpolate_linestring(linestring, distance);
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence, double distance)
{
  const auto merged_sequence = lanelet::LaneletSequence(lanelet_sequence);
  const auto centerline = merged_sequence.centerline();
  return interpolate_linestring(centerline, distance);
  ;
}

std::optional<lanelet::CompoundLineString3d> concatenate_center_line(
  const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }
  const auto merged_sequence = lanelet::LaneletSequence(lanelets);
  return merged_sequence.centerline();
}

std::optional<lanelet::ConstLineString3d> get_linestring_from_arc_length(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::ConstPoints3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = lanelet::geometry::length(linestring);
  if (s1 < 0.0 || s2 > total_length || s1 >= s2) {
    return std::nullopt;
  }
  const size_t last_linestring_idx = linestring.size() - 1;

  for (size_t i = 0; i < last_linestring_idx; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = lanelet::geometry::distance3d(p1, p2);

    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }

  if (start_index < last_linestring_idx) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;

    const auto start_point = interpolate_point(p1, p2, residue);
    if (!start_point.has_value()) return std::nullopt;
    points.emplace_back(start_point.value());
  }

  size_t end_index = linestring.size();
  for (size_t i = start_index; i < last_linestring_idx; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = lanelet::geometry::distance3d(p1, p2);
    if (accumulated_length + length > s2) {
      end_index = i;
      break;
    }
    accumulated_length += length;
  }

  for (size_t i = start_index + 1; i < end_index; i++) {
    const auto p = lanelet::Point3d(linestring[i]);
    points.emplace_back(p);
  }
  if (end_index < last_linestring_idx) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto end_point = interpolate_point(p1, p2, residue);
    points.emplace_back(linestring[end_index]);

    if (!end_point.has_value()) return std::nullopt;

    points.emplace_back(lanelet::InvalId, end_point.value());
  }
  return create_safe_linestring(points);
}

std::optional<geometry_msgs::msg::Pose> get_pose_from_2d_arc_length(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0.0;

  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    for (auto it = centerline.begin(); std::next(it) != centerline.end(); ++it) {
      const auto & pt = *it;
      const auto & next_pt = *std::next(it);
      const double distance2d = lanelet::geometry::distance3d(pt, next_pt);

      if (accumulated_distance2d + distance2d > s) {
        double rem = s - accumulated_distance2d;
        auto const_pt = interpolate_point(pt, next_pt, rem);
        if (!const_pt.has_value()) {
          return std::nullopt;
        }
        auto P = const_pt.value().basicPoint();

        double half_yaw = std::atan2(next_pt.y() - pt.y(), next_pt.x() - pt.x()) * 0.5;

        geometry_msgs::msg::Pose pose;
        pose.position.x = P.x();
        pose.position.y = P.y();
        pose.position.z = P.z();
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(half_yaw);
        pose.orientation.w = std::cos(half_yaw);

        return pose;
      }

      accumulated_distance2d += distance2d;
    }
  }
  return std::nullopt;
}

lanelet::ConstLineString3d get_closest_segment(
  const lanelet::ConstLineString3d & linestring, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d closest_segment;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & [prev_pt, current_pt] :
       ranges::views::zip(linestring, linestring | ranges::views::drop(1))) {
    lanelet::Point3d prev_pt_3d(lanelet::InvalId, prev_pt.x(), prev_pt.y(), prev_pt.z());
    lanelet::Point3d current_pt_3d(
      lanelet::InvalId, current_pt.x(), current_pt.y(), current_pt.z());

    lanelet::ConstLineString3d current_segment(lanelet::InvalId, {prev_pt_3d, current_pt_3d});
    double distance = lanelet::geometry::distance3d(current_segment.basicLineString(), search_pt);
    if (distance < min_distance) {
      closest_segment = current_segment;
      min_distance = distance;
    }
  }
  return closest_segment;
}

double get_lanelet_angle(
  const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d segment = get_closest_segment(lanelet.centerline(), search_pt);
  return std::atan2(
    segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
}

geometry_msgs::msg::Pose get_closest_center_pose(
  const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d segment = get_closest_segment(lanelet.centerline(), search_pt);
  if (segment.empty()) {
    geometry_msgs::msg::Pose closest_pose;
    closest_pose.position = to_ros(lanelet.centerline().front(), search_pt.z());
    closest_pose.orientation.x = 0.0;
    closest_pose.orientation.y = 0.0;
    closest_pose.orientation.z = 0.0;
    closest_pose.orientation.w = 1.0;
    return closest_pose;
  }

  const Eigen::Vector2d direction(
    (segment.back().basicPoint2d() - segment.front().basicPoint2d()).normalized());
  const Eigen::Vector2d xf(segment.front().basicPoint2d());
  const Eigen::Vector2d x(search_pt.x(), search_pt.y());
  const Eigen::Vector2d p = xf + (x - xf).dot(direction) * direction;

  geometry_msgs::msg::Pose closest_pose;
  closest_pose.position = to_ros(p, search_pt.z());

  const double lane_yaw = get_lanelet_angle(lanelet, search_pt);
  closest_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(lane_yaw);

  return closest_pose;
}

lanelet::ArcCoordinates get_arc_coordinates(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & pose)
{
  // Handle empty Input (Return default ArcCoordinates)
  if (lanelets.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_lanelet2_utility"),
      "Input lanelets is empty. Returning default ArcCoordinates (length=0, distance=0).");
    return lanelet::ArcCoordinates();
  }
  const auto lanelet_sequence = lanelet::LaneletSequence(lanelets);
  const auto centerline_2d = lanelet_sequence.centerline2d();

  const auto lanelet_point = from_ros(pose);

  lanelet::ArcCoordinates arc_coordinates = lanelet::geometry::toArcCoordinates(
    centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());

  return arc_coordinates;
}

double get_lateral_distance_to_centerline(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  const auto & centerline_2d = lanelet::utils::to2D(lanelet.centerline());
  const auto lanelet_point = from_ros(pose);
  return lanelet::geometry::signedDistance(
    centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
}

double get_lateral_distance_to_centerline(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose)
{
  lanelet::ConstLanelet closest_lanelet = *get_closest_lanelet(lanelet_sequence, pose);

  return get_lateral_distance_to_centerline(closest_lanelet, pose);
}

std::optional<lanelet::ConstLanelet> combine_lanelets_shape(const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }
  const auto add_unique_point =
    [](lanelet::ConstPoints3d & points, const lanelet::ConstPoint3d & new_point) {
      constexpr double distance_threshold = 0.01;
      const auto is_duplicate = std::any_of(
        points.cbegin(), points.cend(),
        [&new_point, distance_threshold](const auto & existing_point) {
          return boost::geometry::distance(existing_point.basicPoint(), new_point.basicPoint()) <=
                 distance_threshold;
        });
      if (!is_duplicate) points.emplace_back(new_point);
    };

  const auto add_unique_points = [&add_unique_point](
                                   lanelet::ConstPoints3d & output, const auto & input_points) {
    std::for_each(
      input_points.begin(), input_points.end(),
      [&output, &add_unique_point](const auto & pt) { add_unique_point(output, pt); });
  };

  lanelet::ConstPoints3d lefts, rights, centers;
  for (const auto & llt : lanelets) {
    add_unique_points(lefts, llt.leftBound());
    add_unique_points(rights, llt.rightBound());
    add_unique_points(centers, llt.centerline());
  }

  const auto combined_lanelet_opt = create_safe_lanelet(lefts, rights);
  assert(combined_lanelet_opt.has_value() && "lefts or rights bound size is less than 2.");
  auto combined_lanelet = remove_const(*combined_lanelet_opt);

  const auto center_line_opt = create_safe_linestring(centers);
  assert(center_line_opt.has_value() && "centers size is less than 2.");
  const auto center_line = remove_const(*center_line_opt);

  combined_lanelet.setCenterline(center_line);
  return combined_lanelet;
}

std::optional<lanelet::ConstLanelet> get_dirty_expanded_lanelet(
  const lanelet::ConstLanelet & lanelet_obj, const double left_offset, const double right_offset)
{
  if (left_offset < 0 || right_offset > 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_lanelet2_utility"),
      "Invalid offsets: left_offset must be >= 0, right_offset must be <= 0");
    return std::nullopt;
  }
  const auto copy_z = [](const lanelet::ConstLineString3d & from, lanelet::Points3d & to) {
    lanelet::Points3d new_to = to;
    if (from.empty() || to.empty()) return to;
    new_to.front().z() = from.front().z();
    if (from.size() < 2 || to.size() < 2) return new_to;
    new_to.back().z() = from.back().z();
    auto i_from = 1lu;
    auto s_from = lanelet::geometry::distance2d(from[0], from[1]);
    auto s_to = 0.0;
    auto s_from_prev = 0.0;
    for (auto i_to = 1lu; i_to + 1 < to.size(); ++i_to) {
      s_to += lanelet::geometry::distance2d(new_to[i_to - 1], new_to[i_to]);
      for (; s_from < s_to && i_from + 1 < from.size(); ++i_from) {
        s_from_prev = s_from;
        s_from += lanelet::geometry::distance2d(from[i_from], from[i_from + 1]);
      }
      const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
      new_to[i_to].z() = from[i_from - 1].z() + ratio * (from[i_from].z() - from[i_from - 1].z());
    }
    return new_to;
  };

  const auto to_points3d = [](const lanelet::BasicLineString2d & ls2d) {
    lanelet::Points3d output;
    for (const auto & pt : ls2d) {
      output.push_back(lanelet::Point3d(lanelet::InvalId, pt.x(), pt.y(), 0.0));
    }
    return output;
  };

  using lanelet::geometry::offsetNoThrow;
  using lanelet::geometry::internal::checkForInversion;

  const auto & orig_left_bound_2d = lanelet_obj.leftBound2d().basicLineString();
  const auto & orig_right_bound_2d = lanelet_obj.rightBound2d().basicLineString();

  // Note: The lanelet::geometry::offset throws exception when the undesired inversion is found.
  // Use offsetNoThrow until the logic is updated to handle the inversion.
  // TODO(Horibe) update
  // Note: this is ported from autoware_lanelet2_extension
  auto expanded_left_bound_2d = offsetNoThrow(orig_left_bound_2d, left_offset);
  auto expanded_right_bound_2d = offsetNoThrow(orig_right_bound_2d, right_offset);

  rclcpp::Clock clock{RCL_ROS_TIME};
  try {
    checkForInversion(orig_left_bound_2d, expanded_left_bound_2d, left_offset);
    checkForInversion(orig_right_bound_2d, expanded_right_bound_2d, right_offset);
  } catch (const lanelet::GeometryError & e) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("autoware_lanelet2_utility"), clock, 1000,
      "Fail to expand lanelet. output may be undesired. Lanelet points interval in map data could "
      "be too narrow.");
  }

  lanelet::Points3d ex_lefts = to_points3d(expanded_left_bound_2d);
  lanelet::Points3d ex_rights = to_points3d(expanded_right_bound_2d);
  copy_z(lanelet_obj.leftBound3d(), ex_lefts);
  copy_z(lanelet_obj.rightBound3d(), ex_rights);

  auto convert_to_const = [](const lanelet::Points3d from) {
    lanelet::ConstPoints3d to;
    for (const auto & pt : from) {
      to.emplace_back(pt);
    }
    return to;
  };

  auto const_ex_lefts = convert_to_const(ex_lefts);
  auto const_ex_rights = convert_to_const(ex_rights);
  const auto & lanelet_opt = create_safe_lanelet(const_ex_lefts, const_ex_rights);
  assert(lanelet_opt.has_value() && "lefts or rights bound size is less than 2.");

  // return optional value
  return lanelet_opt;
}

std::optional<lanelet::ConstLanelets> get_dirty_expanded_lanelets(
  const lanelet::ConstLanelets & lanelet_obj, const double left_offset, const double right_offset)
{
  if (left_offset < 0 || right_offset > 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("autoware_lanelet2_utility"),
      "Invalid offsets: left_offset must be >= 0, right_offset must be <= 0");
    return std::nullopt;
  }
  lanelet::ConstLanelets lanelets;
  for (const auto & llt : lanelet_obj) {
    auto expanded_lanelet_opt = get_dirty_expanded_lanelet(llt, left_offset, right_offset);
    // If at least one lanelet in vector cannot be expanded, return nullopt.
    if (!expanded_lanelet_opt.has_value()) {
      return std::nullopt;
    }
    lanelets.push_back(expanded_lanelet_opt.value());
  }
  return lanelets;
}

lanelet::ConstLineString3d get_centerline_with_offset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // get number of segments from resolution and longer bound
  const auto num_segments = compute_num_segments(lanelet_obj, resolution);

  // Resample points
  const auto left_points = resample_points(lanelet_obj.leftBound().basicLineString(), num_segments);
  const auto right_points =
    resample_points(lanelet_obj.rightBound().basicLineString(), num_segments);

  lanelet::ConstPoints3d center_points;
  for (size_t i = 0; i < num_segments + 1; i++) {
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;

    const auto vec_right_2_left = (left_points.at(i) - right_points.at(i)).normalized();

    const auto offset_center_basic_point = center_basic_point + vec_right_2_left * offset;

    const lanelet::ConstPoint3d center_point(
      lanelet::InvalId, offset_center_basic_point.x(), offset_center_basic_point.y(),
      offset_center_basic_point.z());
    center_points.push_back(center_point);
  }

  const auto centerline_opt = create_safe_linestring(center_points);
  assert(centerline_opt.has_value() && "center_points has less than two points.");

  return *centerline_opt;
}

lanelet::ConstLineString3d get_right_bound_with_offset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // get number of segments from resolution and longer bound
  const auto num_segments = compute_num_segments(lanelet_obj, resolution);

  // Resample points
  const auto left_points = resample_points(lanelet_obj.leftBound().basicLineString(), num_segments);
  const auto right_points =
    resample_points(lanelet_obj.rightBound().basicLineString(), num_segments);

  lanelet::ConstPoints3d right_bound_points;
  for (size_t i = 0; i < num_segments + 1; i++) {
    const auto vec_left_2_right = (right_points.at(i) - left_points.at(i)).normalized();

    const auto offset_right_basic_point = right_points.at(i) + vec_left_2_right * offset;

    const lanelet::ConstPoint3d right_bound_point(
      lanelet::InvalId, offset_right_basic_point.x(), offset_right_basic_point.y(),
      offset_right_basic_point.z());
    right_bound_points.push_back(right_bound_point);
  }
  const auto right_bound_opt = create_safe_linestring(right_bound_points);
  assert(right_bound_opt.has_value() && "right_bound_points has less than two points.");

  return *right_bound_opt;
}

lanelet::ConstLineString3d get_left_bound_with_offset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // get number of segments from resolution and longer bound
  const auto num_segments = compute_num_segments(lanelet_obj, resolution);

  // Resample points
  const auto left_points = resample_points(lanelet_obj.leftBound().basicLineString(), num_segments);
  const auto right_points =
    resample_points(lanelet_obj.rightBound().basicLineString(), num_segments);

  lanelet::ConstPoints3d left_bound_points;
  for (size_t i = 0; i < num_segments + 1; i++) {
    const auto vec_right_2_left = (left_points.at(i) - right_points.at(i)).normalized();

    const auto offset_left_basic_point = left_points.at(i) + vec_right_2_left * offset;

    const lanelet::ConstPoint3d left_bound_point(
      lanelet::utils::getId(), offset_left_basic_point.x(), offset_left_basic_point.y(),
      offset_left_basic_point.z());
    left_bound_points.push_back(left_bound_point);
  }

  const auto left_bound_opt = create_safe_linestring(left_bound_points);
  assert(left_bound_opt.has_value() && "left_bound_points has less than two points.");

  return *left_bound_opt;
}

}  // namespace autoware::experimental::lanelet2_utils

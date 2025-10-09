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
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <range/v3/view/enumerate.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{

std::optional<lanelet::ConstLanelet> get_closest_lanelet(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }

  const lanelet::BasicPoint3d search_point = from_ros(search_pose);

  lanelet::ConstLanelets candidate_lanelets;
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & llt : lanelets) {
    /*
      NOTE(soblin):
      if point is on the edge of, or within the area of that polygon, distance is
      0.0.
      comparable_distance is a distance that maybe be x*x+y*y, not std::hypot of it,
      which is used to only compare distance
      stackoverflow.com/questions/51267577/boost-geometry-polygon-distance-for-inside-point
     */
    const double distance = boost::geometry::comparable_distance(
      llt.polygon2d().basicPolygon(), lanelet::utils::to2D(search_point));

    /*
      NOTE(soblin): this line is intended to push all lanelets on which search_point is located to
      candidate, and later judge by angle
     */
    if (std::fabs(distance - min_distance) <= std::numeric_limits<double>::epsilon()) {
      candidate_lanelets.push_back(llt);
    } else if (distance < min_distance) {
      candidate_lanelets.clear();
      candidate_lanelets.push_back(llt);
      min_distance = distance;
    }
  }

  if (candidate_lanelets.size() == 1) {
    return candidate_lanelets.front();
  }

  // find by angle
  const double pose_yaw = tf2::getYaw(search_pose.orientation);
  double min_angle = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> closest_lanelet{};
  for (const auto & llt : candidate_lanelets) {
    const lanelet::ConstLineString3d segment = get_closest_segment(llt.centerline(), search_point);
    if (segment.empty()) {
      continue;
    }
    const auto segment_angle = std::atan2(
      segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
    const auto angle_diff =
      std::fabs(autoware_utils_math::normalize_radian(segment_angle - pose_yaw));
    if (angle_diff < min_angle) {
      min_angle = angle_diff;
      closest_lanelet = llt;
    }
  }
  return closest_lanelet;
}

std::optional<lanelet::ConstLanelet> get_closest_lanelet_within_constraint(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose,
  const double dist_threshold, const double yaw_threshold)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }

  const lanelet::BasicPoint3d search_point = from_ros(search_pose);

  std::vector<std::pair<lanelet::ConstLanelet, double>> candidate_lanelets;
  for (const auto & llt : lanelets) {
    const double distance =
      boost::geometry::distance(llt.polygon2d().basicPolygon(), lanelet::utils::to2D(search_point));

    if (distance <= dist_threshold) {
      candidate_lanelets.emplace_back(llt, distance);
    }
  }

  if (candidate_lanelets.empty()) {
    return std::nullopt;
  }

  // sort by distance
  std::sort(
    candidate_lanelets.begin(), candidate_lanelets.end(),
    [](
      const std::pair<lanelet::ConstLanelet, double> & x,
      const std::pair<lanelet::ConstLanelet, double> & y) { return x.second < y.second; });

  // find closest lanelet within yaw_threshold
  const double pose_yaw = tf2::getYaw(search_pose.orientation);
  double min_angle = std::numeric_limits<double>::max();
  double min_distance = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> closest_lanelet;
  for (const auto & llt_pair : candidate_lanelets) {
    const auto & distance = llt_pair.second;

    double lanelet_angle = get_lanelet_angle(llt_pair.first, search_point);
    double angle_diff = std::fabs(autoware_utils_math::normalize_radian(lanelet_angle - pose_yaw));
    // reject
    if (angle_diff > std::fabs(yaw_threshold)) continue;

    // only 1st item
    if (min_distance < distance) break;

    if (angle_diff < min_angle) {
      min_angle = angle_diff;
      min_distance = distance;
      closest_lanelet = llt_pair.first;
    }
  }

  return closest_lanelet;
}

LaneletRTree::LaneletRTree(const lanelet::ConstLanelets & lanelets) : lanelets_(lanelets)
{
  std::vector<Node> nodes;
  nodes.reserve(lanelets.size());
  for (const auto & [i, lanelet] : ranges::views::enumerate(lanelets)) {
    nodes.emplace_back(
      boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(
        lanelet.polygon2d().basicPolygon()),
      i);
  }
  rtree_ = Rtree(nodes);
}

std::optional<lanelet::ConstLanelet> LaneletRTree::get_closest_lanelet(
  const geometry_msgs::msg::Pose search_pose) const
{
  if (lanelets_.empty()) {
    return std::nullopt;
  }
  const auto search_point = lanelet::BasicPoint2d(search_pose.position.x, search_pose.position.y);
  const auto query_nearest = boost::geometry::index::nearest(search_point, lanelets_.size());

  auto min_dist = std::numeric_limits<double>::max();
  lanelet::ConstLanelets candidates;
  for (auto query_it = rtree_.qbegin(query_nearest); query_it != rtree_.qend(); ++query_it) {
    const auto approx_dist_to_lanelet = boost::geometry::distance(search_point, query_it->first);
    if (approx_dist_to_lanelet > min_dist) {
      break;
    }
    const auto dist = boost::geometry::distance(
      search_point, lanelets_.at(query_it->second).polygon2d().basicPolygon());
    if (dist <= min_dist) {
      // NOTE(soblin): if multiple lanelets overlap at same position, they all give zero distance
      candidates.push_back(lanelets_.at(query_it->second));
      min_dist = dist;
    }
  }
  return autoware::experimental::lanelet2_utils::get_closest_lanelet(candidates, search_pose);
}

std::optional<lanelet::ConstLanelet> LaneletRTree::get_closest_lanelet_within_constraint(
  const geometry_msgs::msg::Pose & search_pose, const double dist_threshold,
  const double yaw_threshold) const
{
  const auto pose_yaw = tf2::getYaw(search_pose.orientation);
  const lanelet::BasicPoint3d search_point = from_ros(search_pose);
  const auto query_nearest =
    boost::geometry::index::nearest(lanelet::utils::to2D(search_point), lanelets_.size());

  auto min_dist = std::numeric_limits<double>::max();
  auto min_angle_diff = std::numeric_limits<double>::max();
  std::optional<size_t> optimal_index{};

  for (auto query_it = rtree_.qbegin(query_nearest); query_it != rtree_.qend(); ++query_it) {
    const auto approx_dist_to_lanelet =
      boost::geometry::distance(lanelet::utils::to2D(search_point), query_it->first);
    if (approx_dist_to_lanelet > min_dist || approx_dist_to_lanelet > dist_threshold) {
      break;
    }

    const auto & lanelet = lanelets_.at(query_it->second);
    const auto precise_dist = boost::geometry::distance(
      lanelet::utils::to2D(search_point), lanelet.polygon2d().basicPolygon());
    const auto lanelet_angle = get_lanelet_angle(lanelet, search_point);
    const double angle_diff =
      std::abs(autoware_utils_math::normalize_radian(lanelet_angle - pose_yaw));
    if (precise_dist > dist_threshold || angle_diff > std::abs(yaw_threshold)) {
      continue;
    }
    if (
      precise_dist < min_dist || (precise_dist == min_dist /*nominally both 0.0 if they overlap */
                                  && angle_diff < min_angle_diff)) {
      min_dist = precise_dist;
      min_angle_diff = angle_diff;
      optimal_index = query_it->second;
    }
  }
  if (optimal_index) {
    return lanelets_.at(optimal_index.value());
  }
  return std::nullopt;
}

lanelet::ConstLanelets get_road_lanelets_at(
  const lanelet::LaneletMapConstPtr lanelet_map, const double x, const double y)
{
  const lanelet::BasicPoint2d p{x, y};

  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_at_pose : lanelet_map->laneletLayer.search(lanelet::BoundingBox2d{p})) {
    // NOTE(soblin): for Point, we do not need bbox actually -- RTree returns no false positives
    if (is_road_lane(lanelet_at_pose) && lanelet::geometry::inside(lanelet_at_pose, p)) {
      lanelets.push_back(lanelet_at_pose);
    }
  }
  return lanelets;
}

lanelet::ConstLanelets get_shoulder_lanelets_at(
  const lanelet::LaneletMapConstPtr lanelet_map, const double x, const double y)
{
  const lanelet::BasicPoint2d p{x, y};

  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_at_pose : lanelet_map->laneletLayer.search(lanelet::BoundingBox2d{p})) {
    // NOTE(soblin): for Point, we do not need bbox actually -- RTree returns no false positives
    if (is_shoulder_lane(lanelet_at_pose) && lanelet::geometry::inside(lanelet_at_pose, p)) {
      lanelets.push_back(lanelet_at_pose);
    }
  }
  return lanelets;
}

}  // namespace autoware::experimental::lanelet2_utils

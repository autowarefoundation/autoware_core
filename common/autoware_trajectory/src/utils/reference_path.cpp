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

#include "autoware/trajectory/utils/reference_path.hpp"

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/scope_exit.hpp>

#include <fmt/format.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

struct UserDefinedWaypoint
{
  UserDefinedWaypoint() = delete;
  lanelet::ConstPoint3d point;
  // NOTE(soblin): this point may not exist on defined_lanelet_id
  lanelet::Id defined_lanelet_id;
};

using UserDefinedWaypoints = std::vector<UserDefinedWaypoint>;

struct ReferencePoint
{
  ReferencePoint() = delete;
  lanelet::ConstPoint3d point;
  // NOTE(soblin): this point exists on located_lanelet_id
  lanelet::Id located_lanelet_id;
  std::optional<lanelet::Id> next_lanelet_id{std::nullopt};  // only for border point

  bool is_border_point() const { return next_lanelet_id.has_value(); }
};

using ReferencePoints = std::vector<ReferencePoint>;

struct ReferencePointsChunk
{
  ReferencePoints points;  // the points merged from several lanelet UserDefinedWaypoints
                           // that are close to each other
  const double start_s;    // the s coordinate on the designated lanelet sequence
  double end_s{};          // the s coordinate on the designated lanelet sequence
};

static std::optional<UserDefinedWaypoints> get_user_defined_waypoint(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  if (!lanelet.hasAttribute("waypoints")) {
    return std::nullopt;
  }
  const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
  const auto & waypoints_linestring = lanelet_map->lineStringLayer.get(waypoints_id);

  if (waypoints_linestring.size() < 2) {
    return std::nullopt;
  }
  return waypoints_linestring | ranges::views::transform([&lanelet](const auto & p) {
           return UserDefinedWaypoint{p, lanelet.id()};
         }) |
         ranges::to<std::vector>();
}

bool is_waypoint_inside_lanelet(
  const lanelet::ConstLanelet & defined_lanelet, const lanelet::BasicPoint2d & waypoint)
{
  return lanelet::geometry::inside(defined_lanelet, waypoint);
};

bool is_waypoint_inside_lanelet(
  const lanelet::ConstLanelet & defined_lanelet, const UserDefinedWaypoint & waypoint)
{
  return lanelet::geometry::inside(defined_lanelet, waypoint.point.basicPoint2d());
};

tl::expected<std::pair<ReferencePoints, std::string>, std::string> validate_relaxed_vm_01_11_spec(
  const lanelet::ConstLanelet & defined_lanelet,
  const std::optional<lanelet::ConstLanelet> & prev_lanelet,
  const std::optional<lanelet::ConstLanelet> & next_lanelet, const UserDefinedWaypoints & waypoints)
{
  std::string warn_msg;

  const auto first_waypoint_inside_lanelet_it = std::find_if(
    waypoints.begin(), waypoints.end(),
    [&](const auto & it) { return is_waypoint_inside_lanelet(defined_lanelet, it); });
  const auto last_waypoint_inside_lanelet_it =
    std::next(
      std::find_if(
        waypoints.rbegin(), waypoints.rend(),
        [&](const auto & it) { return is_waypoint_inside_lanelet(defined_lanelet, it); }))
      .base();
  if (
    first_waypoint_inside_lanelet_it == waypoints.end() ||
    last_waypoint_inside_lanelet_it == waypoints.end()) {
    // no waypoint inside lanelet, thus ignore this chunk
    return tl::make_unexpected<std::string>(fmt::format(
      "no custom centerline points of Lanelet {} is within the lane, ignoring",
      defined_lanelet.id()));
  }
  const size_t first_waypoint_inside_idx =
    std::distance(waypoints.begin(), first_waypoint_inside_lanelet_it);
  if (first_waypoint_inside_idx > 2) {
    warn_msg += fmt::format(
      "custom centerline has more than {} points outside of the Lanelet {} in the beginning, "
      "ignoring invalid part",
      first_waypoint_inside_idx, defined_lanelet.id());
  }
  const size_t last_waypoint_inside_idx =
    std::distance(waypoints.begin(), last_waypoint_inside_lanelet_it);
  if (waypoints.size() >= last_waypoint_inside_idx + 3) {
    warn_msg += fmt::format(
      "custom centerline has more than 2 end points outside of the Lanelet {}, ignoring the "
      "centerline",
      defined_lanelet.id());
  }
  for (auto it = first_waypoint_inside_lanelet_it; it != last_waypoint_inside_lanelet_it; it++) {
    if (!is_waypoint_inside_lanelet(defined_lanelet, it->point.basicPoint2d())) {
      return tl::make_unexpected<std::string>(fmt::format(
        "custom centerline point {} is outside the designated Lanelet {}, ignoring the centerline",
        it->point.id(), defined_lanelet.id()));
    }
  }
  const size_t vm_01_11_interval_start =
    first_waypoint_inside_idx == 0 ? 0 : first_waypoint_inside_idx - 1;
  const size_t vm_01_11_interval_end = (last_waypoint_inside_idx == waypoints.size() - 1)
                                         ? last_waypoint_inside_idx
                                         : last_waypoint_inside_idx + 1;

  ReferencePoints validated_points;
  for (auto i = vm_01_11_interval_start; i <= vm_01_11_interval_end; i++) {
    const auto & waypoint = waypoints.at(i);
    if (is_waypoint_inside_lanelet(defined_lanelet, waypoint)) {
      validated_points.push_back(ReferencePoint{waypoint.point, waypoint.defined_lanelet_id});
      continue;
    }
    if (i == vm_01_11_interval_start && prev_lanelet) {
      validated_points.push_back(ReferencePoint{waypoint.point, prev_lanelet->id()});
      continue;
    }
    if (i == vm_01_11_interval_end && next_lanelet) {
      validated_points.push_back(ReferencePoint{waypoint.point, next_lanelet->id()});
    }
  }
  return std::make_pair(validated_points, warn_msg);
}

class UserDefinedReferencePointsGroup
{
private:
  /**
   * @brief if the interval of points is [s1, s2] for example, extend [s1, s2] by some margin, like
   * [s1 - margin, s2 + margin]. read the document for more detail of the geometric description
   * @param current_lanelet_distance_from_route_start defined_lanelet's cumulative distance on the
   * designated lanelet_sequence
   * @param vm_01_11_validated_points the point meets vm-01-11 map specification
   * @param defined_lanelet the lanelet on which points is defined
   */
  static std::tuple<double, double> compute_smooth_interval(
    const ReferencePoints & vm_01_11_validated_points,
    const double current_lanelet_distance_from_route_start,
    const lanelet::ConstLanelet & defined_lanelet,
    const double connection_from_default_point_gradient)
  {
    const auto compute_remainder_frenet_for_outside_lanelet_point =
      [](
        const lanelet::BasicLineString2d & line,
        const lanelet::BasicPoint2d & point) -> std::pair<double, double> {
      const double length = boost::geometry::distance(line, point);
      const auto & left_point = line.front();
      const auto & right_point = line.back();
      const auto middle_point = (left_point + right_point) / 2.0;
      const double hypot = (middle_point - point).norm();
      const bool is_closer_to_left = (left_point - point).norm() < (right_point - point).norm();
      const double sign = is_closer_to_left ? 1.0 : -1.0;
      const double distance =
        hypot > length ? sign * std::sqrt(hypot * hypot - length * length) : 0.0;
      return {length, distance};
    };
    const auto front_frenet_coords = [&]() {
      const auto & first_waypoint = vm_01_11_validated_points.front().point.basicPoint2d();
      if (is_waypoint_inside_lanelet(defined_lanelet, first_waypoint)) {
        return lanelet::geometry::toArcCoordinates(defined_lanelet.centerline2d(), first_waypoint);
      }
      const auto & entry_left_point = defined_lanelet.leftBound().front().basicPoint2d();
      const auto & entry_right_point = defined_lanelet.rightBound().front().basicPoint2d();
      const auto entry_line = lanelet::BasicLineString2d{entry_left_point, entry_right_point};
      const auto [length, distance] =
        compute_remainder_frenet_for_outside_lanelet_point(entry_line, first_waypoint);
      return lanelet::ArcCoordinates{-length, distance};
    }();

    const auto back_frenet_coords = [&]() {
      const auto & last_waypoint = vm_01_11_validated_points.back().point.basicPoint2d();
      if (is_waypoint_inside_lanelet(defined_lanelet, last_waypoint)) {
        return lanelet::geometry::toArcCoordinates(defined_lanelet.centerline2d(), last_waypoint);
      }
      const auto & exit_left_point = defined_lanelet.leftBound().back().basicPoint2d();
      const auto & exit_right_point = defined_lanelet.rightBound().back().basicPoint2d();
      const auto exit_line = lanelet::BasicLineString2d{exit_left_point, exit_right_point};
      const auto [length, distance] =
        compute_remainder_frenet_for_outside_lanelet_point(exit_line, last_waypoint);
      return lanelet::ArcCoordinates{
        lanelet::geometry::length2d(defined_lanelet) + length, distance};
    }();

    const double first =
      current_lanelet_distance_from_route_start + front_frenet_coords.length -
      connection_from_default_point_gradient * std::fabs(front_frenet_coords.distance);
    const double second =
      current_lanelet_distance_from_route_start + back_frenet_coords.length +
      connection_from_default_point_gradient * std::fabs(back_frenet_coords.distance);
    return {first, second};
  }

  std::vector<ReferencePointsChunk> reference_points_chunks_;

public:
  void add(
    ReferencePoints user_defined_reference_points /* more than 2 */,
    const double current_lanelet_distance_from_route_start,
    const lanelet::ConstLanelet & defined_lanelet, const double group_separation_distance,
    const double connection_from_default_point_gradient)
  {
    if (reference_points_chunks_.empty()) {
      auto [start, end] = compute_smooth_interval(
        user_defined_reference_points, current_lanelet_distance_from_route_start, defined_lanelet,
        connection_from_default_point_gradient);
      if (start < 0.0) {
        start = 0.0;
      }
      reference_points_chunks_.push_back({std::move(user_defined_reference_points), start, end});
      return;
    }

    if (const auto & last_waypoints = reference_points_chunks_.back().points;
        lanelet::geometry::distance3d(
          last_waypoints.back().point, user_defined_reference_points.front().point) >
        group_separation_distance) {
      const auto [start, end] = compute_smooth_interval(
        user_defined_reference_points, current_lanelet_distance_from_route_start, defined_lanelet,
        connection_from_default_point_gradient);
      reference_points_chunks_.push_back({std::move(user_defined_reference_points), start, end});
      return;
    }

    /*
      '+' are ConstPoint3d

      ----> lane direction

      if last_waypoints and next_waypoints are close, next_waypoints are merged into last_waypoints

      >+--+--+--+--+ last_waypoints --+--+--+--+--+> >+--+--+--+--+ next_waypoints --+--+--+--+--+>
     */
    auto & last_waypoints_mut = reference_points_chunks_.back().points;
    // NOTE(soblin): if two custom centerlines are subsequent, waypoints can be duplicate
    for (const auto & user_defined_reference_point : user_defined_reference_points) {
      const auto last_added_point_id = last_waypoints_mut.back().point.id();
      const auto to_add_point_id = user_defined_reference_point.point.id();
      if (last_added_point_id != to_add_point_id) {
        last_waypoints_mut.push_back(user_defined_reference_point);
      }
    }
    const auto [_, new_end] = compute_smooth_interval(
      user_defined_reference_points, current_lanelet_distance_from_route_start, defined_lanelet,
      connection_from_default_point_gradient);
    reference_points_chunks_.back().end_s = new_end;
  }

  std::vector<ReferencePointsChunk> reference_points_chunks()
  {
    return std::move(reference_points_chunks_);
  }
};

static std::vector<std::pair<lanelet::ConstLanelet, double>> accumulate_distance(
  const lanelet::ConstLanelets & lanelet_sequence)
{
  std::vector<std::pair<lanelet::ConstLanelet, double>> ll_with_dist;
  double acc_dist = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    ll_with_dist.emplace_back(lanelet, acc_dist);
    acc_dist += lanelet::geometry::length2d(lanelet);
  }
  return ll_with_dist;
}

static double measure_point_s_no_check(
  const std::vector<std::pair<lanelet::ConstLanelet, double>> & lanelet_with_acc_dist_sequence,
  const ReferencePoint & waypoint)
{
  const auto & point = waypoint.point;
  const auto & lane_id = waypoint.located_lanelet_id;
  const auto [lanelet, acc_dist] = *std::find_if(
    lanelet_with_acc_dist_sequence.begin(), lanelet_with_acc_dist_sequence.end(),
    [&](const auto & ll) { return ll.first.id() == lane_id; });
  return acc_dist +
         lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point.basicPoint2d()).length;
};

static ReferencePoints sanitize_and_crop(
  const std::vector<ReferencePoint> & inputs,
  const std::vector<std::pair<lanelet::ConstLanelet, double>> & lanelet_with_acc_dist_sequence,
  const double s_start, const double s_end)
{
  auto s_with_ref =
    inputs |
    ranges::views::transform(
      [&](const auto & point) -> std::pair<double, std::reference_wrapper<const ReferencePoint>> {
        return std::make_pair(
          measure_point_s_no_check(lanelet_with_acc_dist_sequence, point), std::cref(point));
      }) |
    ranges::to<std::vector>();
  // sanitize invalid points so that s is monotonic
  for (auto it = s_with_ref.begin(); it != s_with_ref.end();) {
    if (it == s_with_ref.begin()) {
      it++;
      continue;
    }
    if (it->first < std::prev(it)->first) {
      // s is not monotonic
      if (it->second.get().is_border_point()) {
        // border point must be preserved
        it = s_with_ref.erase(std::prev(it));
      } else {
        it = s_with_ref.erase(it);
      }
    } else {
      it++;
    }
  }
  if (s_with_ref.size() < 2) {
    return {};
  }

  // now s_with_ref is monotonic, so take the element
  auto s_start_it = std::lower_bound(
    s_with_ref.begin(), s_with_ref.end(),
    std::make_pair(s_start /*search for element >= this */, inputs.front() /*dummy*/),
    [](
      const std::pair<double, const ReferencePoint &> & a,
      const std::pair<double, const ReferencePoint &> & b) { return a.first < b.first; });
  if (s_start_it != s_with_ref.begin()) {
    s_start_it = std::prev(s_start_it);
  }

  auto s_end_it = std::lower_bound(
    s_with_ref.begin(), s_with_ref.end(),
    std::make_pair(s_end /* search for element >= this */, inputs.front() /*dummy*/),
    [](
      const std::pair<double, const ReferencePoint &> & a,
      const std::pair<double, const ReferencePoint &> & b) { return a.first < b.first; });

  ReferencePoints monotonic_reference_points;
  for (auto it = s_start_it; it != s_with_ref.end(); ++it) {
    monotonic_reference_points.push_back(it->second);
    if (it == s_end_it) {
      break;
    }
  }
  return monotonic_reference_points;
}

static bool has_passed_lanelet_border(
  const ReferencePoint & prev_point, const lanelet::Id adding_point_located_lanelet_id)
{
  if (prev_point.is_border_point()) {
    return prev_point.next_lanelet_id.value() != adding_point_located_lanelet_id;
  }
  return prev_point.located_lanelet_id != adding_point_located_lanelet_id;
}

static std::string append_reference_points_no_check(
  const ReferencePoints & waypoints, const lanelet::ConstLanelets & lanelet_sequence,
  ReferencePoints & reference_lanelet_points)
{
  std::string warning;
  for (const auto & [point, located_lane_id, next_lane_id] : waypoints) {
    if (reference_lanelet_points.empty()) {
      reference_lanelet_points.emplace_back(ReferencePoint{point, located_lane_id});
      continue;
    }

    // add border point if lane id changed
    if (const auto prev_reference_point = reference_lanelet_points.back();
        has_passed_lanelet_border(prev_reference_point, located_lane_id)) {
      const auto & [prev_point, prev_located_lane_id, prev_next_lane_id] = prev_reference_point;
      const auto located_lanelet = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [lane_id = located_lane_id](const auto & lane) { return lane.id() == lane_id; });
      const auto lane_border = lanelet::BasicLineString2d{
        located_lanelet->leftBound().front().basicPoint2d(),
        located_lanelet->rightBound().front().basicPoint2d()};
      const auto waypoint_segment =
        lanelet::BasicLineString2d{prev_point.basicPoint2d(), point.basicPoint2d()};
      std::vector<lanelet::BasicPoint2d> intersection;
      boost::geometry::intersection(lane_border, waypoint_segment, intersection);
      if (!intersection.empty()) {
        const auto & border_point_2d = intersection.front();
        const double dist_a = lanelet::geometry::distance2d(border_point_2d, lane_border.front());
        const double dist_b = lanelet::geometry::distance2d(border_point_2d, lane_border.back());
        const double z = (located_lanelet->leftBound().front().z() * dist_b +
                          located_lanelet->rightBound().front().z() * dist_a) /
                         (dist_a + dist_b);
        const ReferencePoint border_point{
          lanelet::ConstPoint3d{
            lanelet::InvalId, lanelet::BasicPoint3d{border_point_2d.x(), border_point_2d.y(), z}},
          prev_located_lane_id, located_lane_id};
        if (is_almost_same(border_point.point, prev_point)) {
          if (!prev_next_lane_id) {
            // newly adding border point is too close to prev_point
            reference_lanelet_points.pop_back();
          } else {
            warning += fmt::format(
              "border point from {0} -> {1} and {1} -> {2} are subsequent and too close, ignoreing "
              "the former one",
              prev_located_lane_id, prev_next_lane_id.value(), located_lane_id);
            reference_lanelet_points.pop_back();
          }
        }
        reference_lanelet_points.emplace_back(border_point);
      } else {
      }
    }

    const auto [prev_point, prev_located_lane_id, _] = reference_lanelet_points.back();
    if (is_almost_same(prev_point, point)) {
      continue;
    }
    reference_lanelet_points.emplace_back(ReferencePoint{point, located_lane_id});
  }
  return warning;
}

/**
 * @param waypoint_chunks WaypointGroupChunk defined on the lanelet_sequence
 * @param lanelet_sequence consecutive lanelet sequence
 * @param s_start On the interval of [0.0, length(lanelet_sequence)], trim the path from s_start
 * @param s_end On the interval of [0.0, length(lanelet_sequence)], trim the path from s_end
 * @pre s_start < s_end
 */
static std::pair<ReferencePoints, std::string>
consolidate_user_defined_waypoints_and_native_centerline(
  const std::vector<ReferencePointsChunk> & reference_points_chunks,
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end)
{
  std::string warning;
  // reference path generation algorithm
  //
  // Basically, each native_point in centerline of original lanelet_sequence is added to
  // reference_lanelet_points, but if a native_point is within the interval of
  // current_overlapped_chunk_iter, the waypoints on that chunk are added instead.
  // s_start, and s_end are measured by the accumulated distance of native points in
  // lanelet_sequence
  std::vector<ReferencePoint> reference_lanelet_points;
  const auto lanelet_with_acc_dist_sequence = accumulate_distance(lanelet_sequence);

  /*
    Main Algorithm:

    Basically this algorithm iterates `native` points, or the points defined on the original
    Lanelet, by the iterator named `native_point_it`. `prev_point_it` saves its previous value

    At the beginning of every iteration, its s-value may start overlapping
    `current_target_overlapped_chunk_iter`. In that case, the whole chunk is added,
    `is_overlapping_target_chunk` turns true.
    If its s-value exceeded `current_target_overlapped_chunk_iter->end`,
    `is_overlapping_target_chunk` turns false and `current_target_overlapped_chunk_iter` is
    incremented.

    While the s-value of `native_point_it` is less than the interval start of
    `current_target_overlapped_chunk_iter`, `native_point_it` is
    - added to `reference_points` if `is_overlapping_target_chunk` is false
    - not added
   */

  // flag to indicate that native point iteration has not passed current_overlapped_chunk_iter yet,
  // which is important if current_overlapped_chunk_iter "steps over" to next lanelet
  std::optional<ReferencePointsChunk> current_overlapping_chunk_iter{std::nullopt};
  auto next_target_chunk_iter = reference_points_chunks.begin();
  std::optional<ReferencePoint> prev_native_point_it{};

  for (const auto & [lanelet, this_lanelet_s] : lanelet_with_acc_dist_sequence) {
    const auto & centerline = lanelet.centerline();
    const auto this_lane_id = lanelet.id();

    bool terminated = false;
    for (auto [native_point_it, native_s] = std::make_tuple(centerline.begin(), this_lanelet_s);
         native_point_it != centerline.end(); native_point_it++) {
      // ensure `prev_native_point_it` is updated
      BOOST_SCOPE_EXIT((&prev_native_point_it)(&native_point_it)(&this_lane_id))
      {
        prev_native_point_it.emplace(ReferencePoint{*native_point_it, this_lane_id});
      }
      BOOST_SCOPE_EXIT_END

      const double prev_native_s = native_s;
      if (native_point_it != centerline.begin()) {
        native_s += lanelet::geometry::distance3d(*std::prev(native_point_it), *native_point_it);
      }

      /*
       * this block is not appropriate because target_chunk needs to be iterated and **consumed**
       * from the beginning
      if (native_s < s_start) { continue;
      }
      */
      if (native_s > s_end) {
        // add this point to interpolate the precise point at s_end
        warning += append_reference_points_no_check(
          std::vector{ReferencePoint{*native_point_it, this_lane_id}}, lanelet_sequence,
          reference_lanelet_points);
        terminated = true;
        break;
      }

      auto exceeded_start_for_the_1st_time_check =
        [&, native_s = native_s]() -> std::optional<ReferencePoint> {
        if (prev_native_s < s_start && s_start <= native_s && prev_native_point_it) {
          return prev_native_point_it;
        }
        return std::nullopt;
      };

      ReferencePoints points_to_add_native;
      ReferencePoints points_to_add_chunk;
      // 1st, check overlap with chunk
      if (!current_overlapping_chunk_iter) {
        if (
          next_target_chunk_iter == reference_points_chunks.end() ||
          native_s < next_target_chunk_iter->start_s) {
          // previously not overlapping, and still not overlapping
          if (const auto exceeded_start = exceeded_start_for_the_1st_time_check(); exceeded_start) {
            // the position at s_start needs to be interpolated
            points_to_add_native.emplace_back(*exceeded_start);
          }

          // just add native point
          points_to_add_native.emplace_back(ReferencePoint{*native_point_it, this_lane_id});
        } else if (native_s <= next_target_chunk_iter->end_s) {
          // previously not overlapping, and now overlapping!
          current_overlapping_chunk_iter.emplace(*next_target_chunk_iter);

          if (const auto exceeded_start = exceeded_start_for_the_1st_time_check(); exceeded_start) {
            // this is possible if s_start is on the waypoint
            // the position at s_start needs to be interpolated
            points_to_add_native.emplace_back(*exceeded_start);
          }

          // append all the points in current_overlapped_chunk_iter at once
          points_to_add_chunk = current_overlapping_chunk_iter.value().points;
        } else {
          // previously not overlapping, and next native point iterator passed the target. This case
          // is almost impossible
          next_target_chunk_iter++;
        }
      } else {
        if (native_s <= current_overlapping_chunk_iter.value().end_s) {
          // previously overlapping, and still overlapping, skip
        } else {
          // previously overlapping, but now not overlapping with previous one. But maybe already
          // overlapping with next chunk
          next_target_chunk_iter++;
          if (
            next_target_chunk_iter != reference_points_chunks.end() &&
            native_s >= next_target_chunk_iter->start_s) {
            current_overlapping_chunk_iter.emplace(*next_target_chunk_iter);

            if (const auto exceeded_start = exceeded_start_for_the_1st_time_check();
                exceeded_start) {
              // this is possible if s_start is on the waypoint
              // the position at s_start needs to be interpolated
              points_to_add_native.emplace_back(*exceeded_start);
            }

            // append all the points in current_overlapped_chunk_iter at once
            points_to_add_chunk = current_overlapping_chunk_iter.value().points;
          } else {
            points_to_add_native.emplace_back(ReferencePoint{*native_point_it, this_lane_id});
            current_overlapping_chunk_iter = std::nullopt;
          }
        }
      }

      if (!points_to_add_native.empty() && native_s >= s_start) {
        append_reference_points_no_check(
          points_to_add_native, lanelet_sequence, reference_lanelet_points);
      }
      if (!points_to_add_chunk.empty()) {
        if (native_s >= s_start) {
          append_reference_points_no_check(
            points_to_add_chunk, lanelet_sequence, reference_lanelet_points);
          continue;
        }
        if (current_overlapping_chunk_iter && s_start <= current_overlapping_chunk_iter->end_s) {
          // pre: s_start < native_s and overlapping with chunk
          // in this case, the overlapping chunk includes s_start part, so this chunk cannot be
          // ignored.
          append_reference_points_no_check(
            points_to_add_chunk, lanelet_sequence, reference_lanelet_points);
        }
      }
    }

    if (terminated) {
      break;
    }
  }

  if (reference_lanelet_points.size() < 2) {
    warning += "collected reference points are 0 or 1";
    return {{}, warning};
  }

  auto monotonic_reference_points =
    sanitize_and_crop(reference_lanelet_points, lanelet_with_acc_dist_sequence, s_start, s_end);
  if (monotonic_reference_points.size() < 2) {
    warning += "collected reference points are 0 or 1";
    return {{}, warning};
  }
  {
    const auto p1 = monotonic_reference_points.front();
    const auto p2 = monotonic_reference_points.at(1);
    const double s_p1 = measure_point_s_no_check(lanelet_with_acc_dist_sequence, p1);
    const double s_p2 = measure_point_s_no_check(lanelet_with_acc_dist_sequence, p2);
    if (s_p1 <= s_start && s_start < s_p2) {
      const auto a = s_start - s_p1;
      const auto b = s_p2 - s_start;
      const auto precise_point = (p1.point.basicPoint() * b + p2.point.basicPoint() * a) / (a + b);
      const auto lane_id =
        p1.is_border_point() ? p1.next_lanelet_id.value() : p1.located_lanelet_id;
      monotonic_reference_points.front() =
        ReferencePoint{lanelet::ConstPoint3d(lanelet::InvalId, precise_point), lane_id};
    }
  }
  {
    const auto p1 = monotonic_reference_points.at(monotonic_reference_points.size() - 2);
    const auto p2 = monotonic_reference_points.back();
    const double s_p1 = measure_point_s_no_check(lanelet_with_acc_dist_sequence, p1);
    const double s_p2 = measure_point_s_no_check(lanelet_with_acc_dist_sequence, p2);  // >= s_end
    if (s_p1 <= s_end && s_end < s_p2) {
      const auto a = s_end - s_p1;
      const auto b = s_p2 - s_end;
      const auto precise_point = (p1.point.basicPoint() * b + p2.point.basicPoint() * a) / (a + b);
      const auto lane_id =
        p1.is_border_point() ? p1.next_lanelet_id.value() : p1.located_lanelet_id;
      monotonic_reference_points.back() =
        ReferencePoint{lanelet::ConstPoint3d(lanelet::InvalId, precise_point), lane_id};
    }
  }
  return {monotonic_reference_points, warning};
}

static std::optional<double> compute_s_on_current_route_lanelet(
  const lanelet::ConstLanelets & route_lanelets, const lanelet::ConstLanelet & current_lanelet,
  const geometry_msgs::msg::Pose & ego_pose)
{
  lanelet::ConstLanelets route_lanelets_before_current;
  bool found_current = false;
  for (const auto & route_lanelet : route_lanelets) {
    if (route_lanelet.id() == current_lanelet.id()) {
      found_current = true;
      break;
    }
    route_lanelets_before_current.push_back(route_lanelet);
  }
  if (!found_current) {
    return std::nullopt;
  }
  const double route_length_before_current =
    route_lanelets_before_current.empty()
      ? 0.0
      : lanelet::geometry::length2d(lanelet::LaneletSequence(route_lanelets_before_current));

  const auto ego_s_current_route =
    route_length_before_current +
    lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(current_lanelet.centerline()),
      lanelet::utils::to2D(lanelet2_utils::from_ros(ego_pose.position)))
      .length;
  return ego_s_current_route;
}

struct LaneletSequenceWithRange
{
  lanelet::ConstLanelets lanelet_sequence;
  double s_start;  // cppcheck-suppress unusedStructMember
  double s_end;    // cppcheck-suppress unusedStructMember
};

/**
 * @brief extend given lanelet sequence backward/forward so that s_start, s_end is within
 * output lanelet sequence
 * @detail extended lanelet sequence must contain the previous/next lanelet of `lanelet_sequence`
 * @param s_start start arc length
 * @param s_end end arc length
 * @return extended lanelet sequence, new start arc length, new end arc length
 * @post s_start >= 0.0, s_end <= lanelet::geometry::length2d(lanelet_sequence)
 */
LaneletSequenceWithRange supplement_lanelet_sequence(
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end)
{
  using autoware::experimental::lanelet2_utils::following_lanelets;
  using autoware::experimental::lanelet2_utils::previous_lanelets;

  auto extended_lanelet_sequence = lanelet_sequence;
  auto new_s_start = s_start;
  auto new_s_end = s_end;

  std::set<lanelet::Id> visited_prev_lane_ids{extended_lanelet_sequence.front().id()};
  bool added_first_prev_lane{false};
  while (!added_first_prev_lane || new_s_start < 0.0) {
    const auto previous_lanes = previous_lanelets(extended_lanelet_sequence.front(), routing_graph);
    if (previous_lanes.empty()) {
      new_s_start = std::max(new_s_start, 0.0);
      break;
    }
    // take the longest previous lane to construct underlying lanelets
    const auto longest_previous_lane = *std::max_element(
      previous_lanes.begin(), previous_lanes.end(), [](const auto & lane1, const auto & lane2) {
        return lanelet::geometry::length2d(lane1) < lanelet::geometry::length2d(lane2);
      });
    if (visited_prev_lane_ids.find(longest_previous_lane.id()) != visited_prev_lane_ids.end()) {
      // loop detected
      break;
    }
    extended_lanelet_sequence.insert(extended_lanelet_sequence.begin(), longest_previous_lane);
    visited_prev_lane_ids.insert(longest_previous_lane.id());
    new_s_start += lanelet::geometry::length2d(longest_previous_lane);
    new_s_end += lanelet::geometry::length2d(longest_previous_lane);
    added_first_prev_lane = true;
  }

  std::set<lanelet::Id> visited_next_lane_ids{extended_lanelet_sequence.back().id()};
  while (new_s_end >
         lanelet::geometry::length2d(lanelet::LaneletSequence(extended_lanelet_sequence))) {
    const auto next_lanes = following_lanelets(extended_lanelet_sequence.back(), routing_graph);
    if (next_lanes.empty()) {
      new_s_end = std::min(
        new_s_end,
        lanelet::geometry::length2d(lanelet::LaneletSequence(extended_lanelet_sequence)));
      break;
    }
    // take the longest previous lane to construct underlying lanelets
    const auto longest_next_lane = *std::max_element(
      next_lanes.begin(), next_lanes.end(), [](const auto & lane1, const auto & lane2) {
        return lanelet::geometry::length2d(lane1) < lanelet::geometry::length2d(lane2);
      });
    if (visited_next_lane_ids.find(longest_next_lane.id()) != visited_next_lane_ids.end()) {
      // loop detected
      break;
    }
    visited_next_lane_ids.insert(longest_next_lane.id());
    extended_lanelet_sequence.push_back(longest_next_lane);
  }

  return {extended_lanelet_sequence, new_s_start, new_s_end};
}

tl::expected<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
build_reference_path(
  const lanelet::ConstLanelets & connected_lane_sequence,
  const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Pose & ego_pose,
  const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules, const double forward_length,
  const double backward_length)
{
  static constexpr double waypoint_group_separation_distance = 1.0;
  static constexpr double waypoint_connection_from_default_point_gradient = 10.0;

  const auto ego_s_current_route_opt =
    compute_s_on_current_route_lanelet(connected_lane_sequence, current_lanelet, ego_pose);
  if (!ego_s_current_route_opt) {
    return tl::make_unexpected<std::string>(
      fmt::format("given ego_pose is not on current_lanelet {}", current_lanelet.id()));
  }
  const auto ego_s_current_route = ego_s_current_route_opt.value();

  const auto [lanelet_sequence, s_start, s_end] = supplement_lanelet_sequence(
    routing_graph, connected_lane_sequence, ego_s_current_route - backward_length,
    ego_s_current_route + forward_length);

  if (s_end <= s_start) {
    return tl::make_unexpected<std::string>(
      fmt::format("given interval does not meet s_start < s_end"));
  }

  UserDefinedReferencePointsGroup waypoint_group;
  std::string warning;
  for (auto [lanelet, acc_dist] = std::make_tuple(lanelet_sequence.begin(), 0.0);
       lanelet != lanelet_sequence.end(); ++lanelet) {
    if (lanelet != lanelet_sequence.begin()) {
      acc_dist += lanelet::geometry::length2d(*std::prev(lanelet));
    }
    const std::optional<lanelet::ConstLanelet> prev_lanelet =
      lanelet == lanelet_sequence.begin()
        ? std::nullopt
        : std::make_optional<lanelet::ConstLanelet>(*std::prev(lanelet));
    const std::optional<lanelet::ConstLanelet> next_lanelet =
      std::next(lanelet) == lanelet_sequence.end()
        ? std::nullopt
        : std::make_optional<lanelet::ConstLanelet>(*std::next(lanelet));
    if (auto user_defined_waypoint_opt = get_user_defined_waypoint(*lanelet, lanelet_map);
        user_defined_waypoint_opt) {
      const auto validation = validate_relaxed_vm_01_11_spec(
        *lanelet, prev_lanelet, next_lanelet, user_defined_waypoint_opt.value());
      if (!validation) {
        warning += validation.error();
      } else {
        const auto & [validated_points, warn] = validation.value();
        warning += warn;
        waypoint_group.add(
          validated_points, acc_dist, *lanelet, waypoint_group_separation_distance,
          waypoint_connection_from_default_point_gradient);
      }
    }
  }
  const auto reference_points_chunks = waypoint_group.reference_points_chunks();

  const auto [reference_lanelet_points, warn] =
    consolidate_user_defined_waypoints_and_native_centerline(
      reference_points_chunks, lanelet_sequence, s_start, s_end);
  warning += warn;

  const auto path_points_with_lane_ids =
    reference_lanelet_points | ranges::views::transform([&](const auto & reference_point) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
      {
        // position
        point.point.pose.position = lanelet2_utils::to_ros(reference_point.point);
      }
      {
        // longitudinal_velocity
        const auto lane_speed =
          reference_point.is_border_point()
            ? traffic_rules
                ->speedLimit(lanelet_map->laneletLayer.get(reference_point.next_lanelet_id.value()))
                .speedLimit.value()
            : traffic_rules
                ->speedLimit(lanelet_map->laneletLayer.get(reference_point.located_lanelet_id))
                .speedLimit.value();
        point.point.longitudinal_velocity_mps = lane_speed;
      }
      {
        // lane_ids
        point.lane_ids.push_back(reference_point.located_lanelet_id);
        if (reference_point.is_border_point()) {
          point.lane_ids.push_back(reference_point.next_lanelet_id.value());
        }
      }
      return point;
    }) |
    ranges::to<std::vector>();

  if (auto trajectory = pretty_build(path_points_with_lane_ids); trajectory) {
    trajectory->align_orientation_with_trajectory_direction();
    return trajectory.value();
  }
  return tl::make_unexpected(warning);
}

}  // namespace autoware::experimental::trajectory

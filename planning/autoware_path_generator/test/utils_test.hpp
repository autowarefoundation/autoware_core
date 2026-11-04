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

#ifndef UTILS_TEST_HPP_
#define UTILS_TEST_HPP_

#include "../src/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
using experimental::lanelet2_utils::to_ros;

// ===================================================================
// MOCK DATA HELPERS
// (Map bins, routes, odometry, etc.)
// ===================================================================

// ========================= I. HELPER FUNCS =========================

inline std::pair<lanelet::LineString3d, lanelet::LineString3d> prep_common_straight_bounds(
  const double length, const double half_width)
{
  // Left bound
  lanelet::Point3d p1_left(1, 0.0, half_width, 0.0);
  lanelet::Point3d p2_left(2, length, half_width, 0.0);
  lanelet::LineString3d left_bound(10, {p1_left, p2_left});

  // Right bound
  lanelet::Point3d p1_right(3, 0.0, -half_width, 0.0);
  lanelet::Point3d p2_right(4, length, -half_width, 0.0);
  lanelet::LineString3d right_bound(11, {p1_right, p2_right});

  return {left_bound, right_bound};
}

inline lanelet::Lanelet prep_lanelet(
  lanelet::LineString3d & left_bound, lanelet::LineString3d & right_bound,
  const lanelet::Id id = 1000)
{
  lanelet::Lanelet this_lanelet(id, left_bound, right_bound);

  this_lanelet.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
  this_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

  return this_lanelet;
}

inline autoware_map_msgs::msg::LaneletMapBin convert_ros_bin_map(
  const std::shared_ptr<lanelet::LaneletMap> & map_ptr)
{
  autoware_map_msgs::msg::LaneletMapBin map_bin_msg =
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map_ptr);
  map_bin_msg.header.frame_id = "map";

  return map_bin_msg;
}

// ========================= II. MAP BINS =========================

// Deterministic, 100-meter straight map
inline autoware_map_msgs::msg::LaneletMapBin create_mock_common_map_bin()
{
  auto map = std::make_shared<lanelet::LaneletMap>();

  auto [left_bound, right_bound] = prep_common_straight_bounds(100.0, 1.75);
  lanelet::Lanelet mock_lanelet = prep_lanelet(left_bound, right_bound, 1000);

  map->add(mock_lanelet);

  return convert_ros_bin_map(map);
}

// 100-meter straight, dense map
// Can inject an optional turn direction attribute
inline autoware_map_msgs::msg::LaneletMapBin create_mock_dense_map_bin(
  const std::string & turn_direction = "")
{
  auto map = std::make_shared<lanelet::LaneletMap>();

  lanelet::Points3d left_points;
  lanelet::Points3d right_points;

  // Inject dense nodes every 1 meter to provide spline resolution
  int pt_id = 10000;
  for (double x = 0.0; x <= 100.0; x += 1.0) {
    left_points.emplace_back(++pt_id, x, 1.75, 0.0);
    right_points.emplace_back(++pt_id, x, -1.75, 0.0);
  }

  lanelet::LineString3d left_bound(10, left_points);
  lanelet::LineString3d right_bound(11, right_points);

  lanelet::Lanelet dense_lanelet = prep_lanelet(left_bound, right_bound, 1000);

  // Apply turn attribute if requested
  if (!turn_direction.empty()) {
    dense_lanelet.attributes()["turn_direction"] = turn_direction;
  }

  map->add(dense_lanelet);

  return convert_ros_bin_map(map);
}

/*
  * 2. Map with 4 lanelets forming a square loop
  *
  * y (m)
  *
  *  40 ┤           p8 ─────────────── p6
  *     │            │                  │
  *     │            │   L3 (go West)   │
  *  38 ┤           p7 ─────────────── p5
  *     │            │                  │
  *     │ L4         │                  │ L2
  *     │ (go South) │                  │ (go North)
  *     │            │                  │
  *   2 ┼ p1 ────────│────────┼──────── p3
  *     │            │   X    │         │
  *   0 ┼ S  ────────┼────────┼──────── ┼──────▶ x (m)
  *     │            │        │         │
  *  -2 ┼ p2 ────────│────────┼──────── p4
  *                  │   L1 (go East)   │
  * -20 ┤           p10 ────────────── p9
  *                  18      20        38      40      42
  *
  * Well if above comment is confusing (yes it's confusing lol), just look below
  *
                            L3 (go West)
      (X = 20, Y = 40) ◀━━━━━━━━━━━━━━━━━━━━━━━┓ (X = 40, Y = 40)
                      ┃                       ┃
                      ┃                       ┃
                  L4  ┃                       ┃  L2
          (go South)  ┃                       ┃  (go North)
                      ┃                       ┃
                      ┃                       ┃
                      ┃      L1 (go East)     ┃
    S ━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━━━━━━━➤┛ (X = 40, Y = 0)
    (X = 0, Y = 0)     ┃  (Truncated here)
                      ┃  (X = 20, Y = 0)
                      ┃
                      ▼ E (X = 20, Y = -20)
  */
inline autoware_map_msgs::msg::LaneletMapBin create_mock_loop_map_bin()
{
  auto map = std::make_shared<lanelet::LaneletMap>();

  // L1 (East): start
  lanelet::Point3d p1(10001, 0.0, 2.0, 0.0);
  lanelet::Point3d p2(10002, 0.0, -2.0, 0.0);

  // Corner 1: L1 => L2 (go North)
  lanelet::Point3d p3(10003, 38.0, 2.0, 0.0);   // Inner
  lanelet::Point3d p4(10004, 42.0, -2.0, 0.0);  // Outer

  // Corner 2: L2 => L3 (go West)
  lanelet::Point3d p5(10005, 38.0, 38.0, 0.0);  // Inner
  lanelet::Point3d p6(10006, 42.0, 42.0, 0.0);  // Outer

  // Corner 3: L3 => L4 (go South)
  lanelet::Point3d p7(10007, 22.0, 38.0, 0.0);  // Inner
  lanelet::Point3d p8(10008, 18.0, 42.0, 0.0);  // Outer

  // Corner 4: end
  lanelet::Point3d p9(10009, 22.0, -20.0, 0.0);   // Left (East side)
  lanelet::Point3d p10(10010, 18.0, -20.0, 0.0);  // Right (West side)

  lanelet::LineString3d ls_l1_left(10101, {p1, p3});
  lanelet::LineString3d ls_l1_right(10102, {p2, p4});

  lanelet::LineString3d ls_l2_left(10103, {p3, p5});
  lanelet::LineString3d ls_l2_right(10104, {p4, p6});

  lanelet::LineString3d ls_l3_left(10105, {p5, p7});
  lanelet::LineString3d ls_l3_right(10106, {p6, p8});

  lanelet::LineString3d ls_l4_left(10107, {p7, p9});
  lanelet::LineString3d ls_l4_right(10108, {p8, p10});

  map->add(prep_lanelet(ls_l1_left, ls_l1_right, 1001));
  map->add(prep_lanelet(ls_l2_left, ls_l2_right, 1002));
  map->add(prep_lanelet(ls_l3_left, ls_l3_right, 1003));
  map->add(prep_lanelet(ls_l4_left, ls_l4_right, 1004));

  return convert_ros_bin_map(map);
}

/**
 * Map with 3 consecutive lanelets in a straight line
 * Used in turn signal tests (GeneratesTurnSignalWhenApproachingTurn etc.)
 *
 * y (m)
 *
 *  1.75 ┼ p1 ─────────────── p2 ─────────────── p3 ─────────────── p4
 *       │      L1 (1001)    │      L2 (1002)    │      L3 (1003)    │
 *       │     (approach)    │  (turn_direction) │       (exit)      │
 *   0.0 ┼ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼──────▶ x (m)
 *       │                   │                   │                   │
 * -1.75 ┼ p5 ─────────────── p6 ─────────────── p7 ─────────────── p8
 *       0                   20                  40                  60
 *
 * Overview:
 *
 *         [L1: ID 1001]              [L2: ID 1002]              [L3: ID 1003]
 *           (Straight)             (turn_direction)               (Straight)
 * (0, 1.75)             (20, 1.75)             (40, 1.75)             (60, 1.75)
 *    p1 ━━━━━━━━━━━━━━━━━━━━ p2 ━━━━━━━━━━━━━━━━━━━━ p3 ━━━━━━━━━━━━━━━━━━━━ p4
 *       ═══════════════════════   ══════════════════════   ═════════════════════▶
 *       ───▶ ego travel ───▶        ───▶ ego travel ───▶     ───▶ ego travel ───▶
 *    p5 ━━━━━━━━━━━━━━━━━━━━ p6 ━━━━━━━━━━━━━━━━━━━━ p7 ━━━━━━━━━━━━━━━━━━━━ p8
 * (0, -1.75)            (20, -1.75)            (40, -1.75)            (60, -1.75)
 *
 *    |◀────── 20 m ────────▶|◀────── 20 m ────────▶|◀────── 20 m ────────▶|
 *    |◀──────────────────────────── 60 m ─────────────────────────────────▶|
 */
inline autoware_map_msgs::msg::LaneletMapBin create_mock_three_lanelets_map_bin(
  const std::string & turn_direction = "")
{
  auto map = std::make_shared<lanelet::LaneletMap>();

  lanelet::Point3d p1(1, 0.0, 1.75, 0.0);
  lanelet::Point3d p2(2, 20.0, 1.75, 0.0);
  lanelet::Point3d p3(3, 40.0, 1.75, 0.0);
  lanelet::Point3d p4(4, 60.0, 1.75, 0.0);

  lanelet::Point3d p5(5, 0.0, -1.75, 0.0);
  lanelet::Point3d p6(6, 20.0, -1.75, 0.0);
  lanelet::Point3d p7(7, 40.0, -1.75, 0.0);
  lanelet::Point3d p8(8, 60.0, -1.75, 0.0);

  lanelet::LineString3d l1_left(11, {p1, p2});
  lanelet::LineString3d l1_right(12, {p5, p6});
  lanelet::LineString3d l2_left(13, {p2, p3});
  lanelet::LineString3d l2_right(14, {p6, p7});
  lanelet::LineString3d l3_left(15, {p3, p4});
  lanelet::LineString3d l3_right(16, {p7, p8});

  auto lane1 = prep_lanelet(l1_left, l1_right, 1001);
  auto lane2 = prep_lanelet(l2_left, l2_right, 1002);
  if (!turn_direction.empty()) {
    lane2.attributes()["turn_direction"] = turn_direction;
  }
  auto lane3 = prep_lanelet(l3_left, l3_right, 1003);

  map->add(lane1);
  map->add(lane2);
  map->add(lane3);

  return convert_ros_bin_map(map);
}

// ========================= III. ROUTES =========================

// Mock route for straight maps
inline autoware_planning_msgs::msg::LaneletRoute create_mock_route()
{
  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 10.5;
  route.start_pose.position.y = 0.0;
  route.start_pose.orientation.w = 1.0;

  route.goal_pose.position.x = 89.5;
  route.goal_pose.position.y = 0.0;
  route.goal_pose.orientation.w = 1.0;

  autoware_planning_msgs::msg::LaneletSegment segment;
  segment.preferred_primitive.id = 1000;
  segment.preferred_primitive.primitive_type = "lane";

  autoware_planning_msgs::msg::LaneletPrimitive primitive;
  primitive.id = 1000;
  primitive.primitive_type = "lane";
  segment.primitives.push_back(primitive);

  route.segments.push_back(segment);
  return route;
}

// Mock route for loop map
inline autoware_planning_msgs::msg::LaneletRoute create_mock_loop_route()
{
  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 5.5;
  route.start_pose.position.y = 0.0;
  route.start_pose.orientation.w = 1.0;

  route.goal_pose.position.x = 20.0;
  route.goal_pose.position.y = -10.5;
  route.goal_pose.orientation.w = 0.707;
  route.goal_pose.orientation.z = -0.707;

  for (const int id : {1001, 1002, 1003, 1004}) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }
  return route;
}

// Mock route for three lanelets map
inline autoware_planning_msgs::msg::LaneletRoute create_mock_three_lanelets_route()
{
  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position.x = 5.0;
  route.start_pose.position.y = 0.0;
  route.start_pose.orientation.w = 1.0;

  route.goal_pose.position.x = 55.0;
  route.goal_pose.position.y = 0.0;
  route.goal_pose.orientation.w = 1.0;

  for (const int id : {1001, 1002, 1003}) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }
  return route;
}

// ========================= IV. ODOMETRY =========================

inline nav_msgs::msg::Odometry set_start_odom(
  const std::optional<autoware_planning_msgs::msg::LaneletRoute> & route)
{
  nav_msgs::msg::Odometry odom;
  if (route.has_value()) {
    odom.pose.pose = route.value().start_pose;
  }
  odom.header.frame_id = "map";
  odom.twist.twist.linear.x = 5.0;
  return odom;
}

// ================================================================

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle_info_ = vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);

    set_map("autoware_test_utils", "lanelet2_map.osm");
    set_route("autoware_path_generator", "common_route.yaml");
    set_path("autoware_path_generator", "common_path.yaml");
  }

  void set_map(const std::string & package_name, const std::string & map_filename)
  {
    const auto lanelet_map_path =
      autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_filename);
    lanelet_map_bin_ = std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(
      autoware::test_utils::make_map_bin_msg(lanelet_map_path));
    if (lanelet_map_bin_->header.frame_id == "") {
      throw std::runtime_error(
        "Frame ID of the map is empty. The file might not exist or be corrupted:" +
        lanelet_map_path);
    }

    route_manager_.reset();
  }

  void set_route(const std::string & package_name, const std::string & route_filename)
  {
    if (!lanelet_map_bin_) {
      throw std::runtime_error("Map not set");
    }

    const auto route_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, route_filename);
    route_ = autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(
      route_path);
    if (!route_) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
    }

    route_manager_ = experimental::lanelet2_utils::RouteManager::create(
      *lanelet_map_bin_, *route_, geometry_msgs::msg::Pose{});
  }

  void set_path(const std::string & package_name, const std::string & path_filename)
  {
    const auto path_path =
      autoware::test_utils::get_absolute_path_to_route(package_name, path_filename);
    try {
      path_ = autoware::test_utils::parse<
                std::optional<autoware_internal_planning_msgs::msg::PathWithLaneId>>(path_path)
                .value();
    } catch (const std::exception &) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + path_path + ". The file might be corrupted.");
    }
  }

  lanelet::ConstLanelet get_lanelet_closest_to_pose(const geometry_msgs::msg::Pose & pose) const
  {
    const auto closest_lanelet = route_manager_->get_closest_preferred_route_lanelet(pose);
    if (!closest_lanelet) {
      throw std::runtime_error("Failed to get the closest lanelet to the given pose.");
    }
    return *closest_lanelet;
  }

  lanelet::ConstLanelet get_lanelet_from_id(const lanelet::Id id) const
  {
    return route_manager_->lanelet_map_ptr()->laneletLayer.get(id);
  }

  lanelet::ConstLanelets get_lanelets_from_ids(const lanelet::Ids & ids) const
  {
    lanelet::ConstLanelets lanelets(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      lanelets[i] = get_lanelet_from_id(ids[i]);
    }
    return lanelets;
  }

  vehicle_info_utils::VehicleInfo vehicle_info_;
  std::optional<experimental::lanelet2_utils::RouteManager> route_manager_{std::nullopt};

  autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr lanelet_map_bin_{nullptr};
  std::optional<autoware_planning_msgs::msg::LaneletRoute> route_{std::nullopt};
  autoware_internal_planning_msgs::msg::PathWithLaneId path_;
};
}  // namespace autoware::path_generator

#endif  // UTILS_TEST_HPP_

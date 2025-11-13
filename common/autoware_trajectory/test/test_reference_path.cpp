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

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::experimental::trajectory
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

struct ReferencePathTestParam
{
  std::string description;
  std::vector<lanelet::Id> lane_ids;
  lanelet::BasicPoint3d ego_position;
  lanelet::Id current_lanelet_id;
  double backward_length;
  double forward_length;
  double waypoint_connection_gradient_from_centerline;

  bool has_result;
};

std::ostream & operator<<(std::ostream & os, const ReferencePathTestParam & p)
{
  return os << p.description;
}

class ReferencePathTest : public ::testing::TestWithParam<ReferencePathTestParam>
{
protected:
  void set_map(const std::string & map_path)
  {
    lanelet_map_ = lanelet2_utils::load_mgrs_coordinate_map(map_path);
    if (!lanelet_map_) {
      throw std::runtime_error("Failed to load map: " + map_path);
    }

    std::tie(routing_graph_, traffic_rules_) =
      lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_);
  }

  lanelet::ConstLanelets get_lanelets_from_ids(const lanelet::Ids & ids) const
  {
    if (!lanelet_map_) {
      throw std::runtime_error("Map not set");
    }

    lanelet::ConstLanelets lanelets;
    for (const auto & id : ids) {
      lanelets.push_back(lanelet_map_->laneletLayer.get(id));
    }
    return lanelets;
  }

  std::optional<Trajectory<PathPointWithLaneId>> build_reference_path()
  {
    const auto & p = GetParam();

    const auto lanelet_sequence = get_lanelets_from_ids(p.lane_ids);

    return trajectory::build_reference_path(
      lanelet_sequence, lanelet_map_, traffic_rules_, routing_graph_,
      lanelet2_utils::to_ros(p.ego_position), p.current_lanelet_id, p.backward_length,
      p.forward_length, p.waypoint_connection_gradient_from_centerline);
  }

  void check_path_shape(const std::vector<PathPointWithLaneId> & result_points) const
  {
    const auto & p = GetParam();

    const auto combined_lanelet =
      lanelet::utils::combineLaneletsShape(get_lanelets_from_ids(p.lane_ids));

    for (const auto [p1, p2, p3] : ranges::views::zip(
           result_points, result_points | ranges::views::drop(1),
           result_points | ranges::views::drop(2))) {
      EXPECT_GE(autoware_utils_geometry::calc_distance3d(p1, p2), k_points_minimum_dist_threshold)
        << "p1: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p1) << "\np2: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p2);
      EXPECT_TRUE(lanelet::geometry::within(
        lanelet::utils::to2D(experimental::lanelet2_utils::from_ros(p2.point.pose.position)),
        combined_lanelet.polygon2d().basicPolygon()))
        << "p2: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p2);
      EXPECT_LT(
        std::abs(autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))),
        M_PI_2)
        << "p1: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p1) << "\np2: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p2) << "\np3: \n"
        << autoware_internal_planning_msgs::msg::to_yaml(p3);
    }
  }

  void check_waypoints(const std::vector<PathPointWithLaneId> & result_points) const
  {
    const auto & p = GetParam();

    for (const auto & lanelet : get_lanelets_from_ids(p.lane_ids)) {
      if (!lanelet.hasAttribute("waypoints")) {
        continue;
      }

      const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
      const auto user_defined_waypoints = lanelet_map_->lineStringLayer.get(waypoints_id);

      for (const auto & waypoint : user_defined_waypoints | ranges::views::drop_last(1)) {
        EXPECT_TRUE(std::any_of(
          result_points.begin(), result_points.end(),
          [&waypoint](const PathPointWithLaneId & result_point) {
            return lanelet::geometry::distance(
                     waypoint,
                     experimental::lanelet2_utils::from_ros(result_point.point.pose.position)) <
                   k_points_minimum_dist_threshold;
          }))
          << "waypoint: (" << waypoint.x() << ", " << waypoint.y() << ", " << waypoint.z() << ")"
          << std::endl;
      }
    }
  }

  void check_border_points(const std::vector<PathPointWithLaneId> & result_points) const
  {
    const auto & p = GetParam();
    const auto lanelet_sequence = get_lanelets_from_ids(p.lane_ids);

    for (auto lanelet_it = std::next(lanelet_sequence.begin());
         lanelet_it != lanelet_sequence.end(); ++lanelet_it) {
      const lanelet::BasicLineString2d border{
        lanelet_it->leftBound().front().basicPoint2d(),
        lanelet_it->rightBound().front().basicPoint2d()};

      EXPECT_TRUE(std::any_of(
        result_points.begin(), result_points.end(),
        [&](const PathPointWithLaneId & result_point) {
          return lanelet::geometry::distance(
                   border, experimental::lanelet2_utils::from_ros(result_point.point.pose.position)
                             .basicPoint2d()) < k_points_minimum_dist_threshold &&
                 result_point.lane_ids.size() == 2 &&
                 ranges::equal(
                   result_point.lane_ids | ranges::to<std::vector>() | ranges::actions::sort,
                   std::vector{std::prev(lanelet_it)->id(), lanelet_it->id()} |
                     ranges::actions::sort);
        }))
        << "lanelet: " << lanelet_it->id() << std::endl;
    }
  }

  lanelet::LaneletMapConstPtr lanelet_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_;
};

struct DenseCenterlineTest : public ReferencePathTest
{
  void SetUp() override
  {
    ReferencePathTest::SetUp();
    set_map(
      ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils") +
      "/sample_map/dense_centerline/lanelet2_map.osm");
  }
};

TEST_P(DenseCenterlineTest, DenseCenterline)
{
  const auto & p = GetParam();

  const auto result = build_reference_path();

  ASSERT_EQ(result.has_value(), p.has_result);
  if (!p.has_result) {
    return;
  }

  const auto result_points = result->restore();

  check_path_shape(result_points);

  check_waypoints(result_points);

  check_border_points(result_points);
}

INSTANTIATE_TEST_SUITE_P(
  , DenseCenterlineTest,
  ::testing::Values(ReferencePathTestParam{
    "Normal",
    {140, 137, 136, 138, 139, 135},
    {125.059, 96.2981, 100},
    140,
    5.0,
    300.0,
    10.0,
    true}),
  ::testing::PrintToStringParamName{});
}  // namespace autoware::experimental::trajectory

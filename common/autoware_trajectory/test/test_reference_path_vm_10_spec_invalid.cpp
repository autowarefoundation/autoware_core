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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/reference_path.hpp"
#include "test_case.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>

#include <array>
#include <limits>
#include <string>
#include <vector>

#define PLOT 1

#ifdef PLOT
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/visualization.hpp>

#include <fmt/format.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#endif

static constexpr auto inf = std::numeric_limits<double>::infinity();

namespace autoware::experimental
{

static void savefig(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    reference_path,
  const double forward, const double backward, const geometry_msgs::msg::Pose & ego_pose,
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & filename)
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];

  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;
  path_plot_config.lane_id = true;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  path.points = reference_path.restore();
  autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
  ax.set_title(Args(
    fmt::format(
      "forward = {}, backward = {}(actual length = {})", forward, backward,
      reference_path.length())));
  for (const auto & route_lanelet : lanelet_sequence) {
    autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
  }
  ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
  ax.set_aspect(Args("equal"));
  ax.legend();
  ax.grid();
  plt.savefig(Args(filename));
}

class TestWithVM_01_10_12_Map : public ::testing::Test  // NOLINT
{
protected:
  void SetUp() override
  {
    const auto test_case_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("autoware_trajectory")) /
      "test_data/test_reference_path_invalid_01.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet_map_ = lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
    std::tie(routing_graph_, traffic_rules_) =
      autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
        lanelet_map_);

    P0 = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
    P3 = test_case_data.manual_poses.at("P3");
    P4 = test_case_data.manual_poses.at("P4");
  };

  geometry_msgs::msg::Pose P0;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;
  geometry_msgs::msg::Pose P3;
  geometry_msgs::msg::Pose P4;

  lanelet::LaneletMapConstPtr lanelet_map_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_{nullptr};
};

TEST_F(TestWithVM_01_10_12_Map, from_P0_on_entire_lanes)  // NOLINT
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P0;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(60);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = inf;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 5);
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 60);
    ASSERT_EQ(border_point.lane_ids.back(), 57);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P1_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P1;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(57);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = inf;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 5);
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 60);
    ASSERT_EQ(border_point.lane_ids.back(), 57);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P2_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P2;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(58);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = inf;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 5);
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 60);
    ASSERT_EQ(border_point.lane_ids.back(), 57);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P3_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P3;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(58);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = inf;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  // All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 5);
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 60);
    ASSERT_EQ(border_point.lane_ids.back(), 57);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P4_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P4;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(59);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = inf;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 5);
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 60);
    ASSERT_EQ(border_point.lane_ids.back(), 57);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }
  {
    // 1st border point
    // 60 -> 57
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(4);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P1_forward_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P1;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(57);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = 0.0;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 4);
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 57);
    ASSERT_EQ(border_point.lane_ids.back(), 56);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 56);
    ASSERT_EQ(border_point.lane_ids.back(), 58);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(2);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(3);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }
  {
    // 2nd border point
    // 57 -> 56
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }
  {
    // 3rd border point
    // 56 -> 58
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(2);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(3);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P2_forward_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P2;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(58);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = 0.0;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 2);
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P3_forward_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P3;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(58);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = 0.0;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 2);
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 58);
    ASSERT_EQ(border_point.lane_ids.back(), 59);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(1);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }
  {
    // 4th border point
    // 58 -> 59
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(1);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

TEST_F(TestWithVM_01_10_12_Map, from_P4_forward_on_entire_lanes)
{
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};
  const auto ego_pose = P4;
  const auto current_lanelet = lanelet_map_->laneletLayer.get(59);
  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();

  const double forward_length = inf;
  const double backward_length = 0.0;
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, current_lanelet, ego_pose, lanelet_map_, routing_graph_, traffic_rules_,
    forward_length, backward_length);
  ASSERT_TRUE(reference_path_opt.has_value());

  const auto & reference_path = reference_path_opt.value();

  const auto path_points_with_lane_id = reference_path.restore();

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         path_points_with_lane_id, path_points_with_lane_id | ranges::views::drop(1),
         path_points_with_lane_id | ranges::views::drop(2))) {
    EXPECT_TRUE(
      autoware_utils_geometry::calc_distance3d(p1, p2) >=
      autoware::experimental::trajectory::k_points_minimum_dist_threshold);
    EXPECT_TRUE(
      std::fabs(
        autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
  }

  //
  //  All of path points are either non-border point(with 1 lane_id) or border point(with 2 lane_id)
  //
  const auto border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 2; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(border_points.size(), 1);
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(0);
    ASSERT_EQ(border_point.lane_ids.front(), 59);
    ASSERT_EQ(border_point.lane_ids.back(), 55);
  }

  const auto non_border_points =
    path_points_with_lane_id |
    ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_EQ(non_border_points.size(), path_points_with_lane_id.size() - border_points.size());

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }
  {
    // 5th border point
    // 59 -> 55
    const auto & border_point = border_points.at(0);
    ASSERT_FLOAT_EQ(border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) +
      "test_reference_path_invalid_01.svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(reference_path, forward_length, backward_length, ego_pose, lanelet_sequence, filename);
  }
#endif
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
#ifdef PLOT
  pybind11::scoped_interpreter guard{};
#endif
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

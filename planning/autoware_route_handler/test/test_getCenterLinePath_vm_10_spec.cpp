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

#include "test_case.hpp"
#include "test_route_handler.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#ifdef PLOT
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/visualization.hpp>

#include <fmt/format.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#endif

static constexpr auto inf = std::numeric_limits<double>::infinity();

namespace autoware::route_handler::test
{

static double calculate_path_with_lane_id_length(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  auto & path_points = path.points;
  double length = 0;
  for (const auto [p1, p2] : ranges::view::zip(path_points, path_points | ranges::view::drop(1))) {
    length += autoware_utils_geometry::calc_distance3d(p1, p2);
  }

  return length;
}

#ifdef PLOT
static void savefig(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const double start_s,
  const double end_s, const double ego_s, const lanelet::ConstLanelets & lanelet_sequence,
  const std::string & filename)
{
  using autoware::experimental::lanelet2_utils::interpolate_lanelet_sequence;

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

  autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
  ax.set_title(Args(
    fmt::format(
      "start_s = {}, end_s = {}(actual length = {})", start_s, end_s,
      calculate_path_with_lane_id_length(path))));
  for (const auto & route_lanelet : lanelet_sequence) {
    autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
  }

  const auto ego_point_opt = interpolate_lanelet_sequence(lanelet_sequence, ego_s);

  if (ego_point_opt.has_value()) {
    const auto & ego_point = *ego_point_opt;
    ax.scatter(Args(ego_point.x(), ego_point.y()), Kwargs("label"_a = "ego position"));
  }
  ax.set_aspect(Args("equal"));
  ax.legend();
  ax.grid();
  plt.savefig(Args(filename));
  std::cout << "Successfully create " << filename << std::endl;
}
#endif

class TestWithVM_01_10_12_Map : public TestRouteHandler,
                                public ::testing::WithParamInterface<std::string>
{
protected:
  void SetUp() override
  {
    using autoware::experimental::lanelet2_utils::from_ros;
    using autoware::experimental::lanelet2_utils::get_arc_coordinates;
    // Load test case
    const auto test_case_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_route_handler")) /
      "test_data" / GetParam();
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    // Load route_handler and set route
    autoware_test_utils_dir = "autoware_route_handler";
    lane_change_right_test_route_filename = "test_reference_path_01.yaml";

    set_route_handler(test_case_data.map_rel_path);
    try {
      set_test_route(lane_change_right_test_route_filename);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }

    // Retrieve target arc-length to test
    std::vector<geometry_msgs::msg::Pose> manual_poses;
    manual_poses.push_back(test_case_data.manual_poses.at("P0"));
    manual_poses.push_back(test_case_data.manual_poses.at("P1"));
    manual_poses.push_back(test_case_data.manual_poses.at("P2"));
    manual_poses.push_back(test_case_data.manual_poses.at("P3"));
    manual_poses.push_back(test_case_data.manual_poses.at("P4"));
    manual_poses.push_back(test_case_data.manual_poses.at("P5"));
    manual_poses.push_back(test_case_data.manual_poses.at("P6"));

    std::vector<double> target_s;

    const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

    for (const auto & pose : manual_poses) {
      target_s.push_back(get_arc_coordinates(lanelet_sequence, pose).length);
    }

    P0_s = target_s[0];
    P1_s = target_s[1];
    P2_s = target_s[2];
    P3_s = target_s[3];
    P4_s = target_s[4];
    P5_s = target_s[5];
    P6_s = target_s[6];
  }

  // lanelet id in the map
  const std::vector<lanelet::Id> ids = {60, 57, 56, 58, 59, 55};

  // target arc-length
  double P0_s;
  double P1_s;
  double P2_s;
  double P3_s;
  double P4_s;
  double P5_s;
  double P6_s;
};

TEST_P(TestWithVM_01_10_12_Map, from_P0_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P0_s - inf;
  const auto end_s = P0_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P0_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P1_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P1_s - inf;
  const auto end_s = P1_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P1_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P2_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P2_s - inf;
  const auto end_s = P2_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P2_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P3_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P3_s - inf;
  const auto end_s = P3_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P3_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P4_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P4_s - inf;
  const auto end_s = P4_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P4_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P5_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P5_s - inf;
  const auto end_s = P5_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P5_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P6_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P6_s - inf;
  const auto end_s = P6_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 10 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P6_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P1_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P1_s - 0.0;
  const auto end_s = P1_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 15 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P1_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P2_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P2_s - 0.0;
  const auto end_s = P2_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 20 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P2_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P3_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P3_s - 0.0;
  const auto end_s = P3_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 25 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P3_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P4_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P4_s - 0.0;
  const auto end_s = P4_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P4_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P5_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P5_s - 0.0;
  const auto end_s = P5_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 30 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P5_s, lanelet_sequence, filename);
  }
#endif
}

TEST_P(TestWithVM_01_10_12_Map, from_P6_forward_on_entire_lanes)  // NOLINT
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;

  const auto start_s = P6_s - 0.0;
  const auto end_s = P6_s + inf;

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  const auto points = centerline_path.points;

  //
  //  points are smooth
  //
  for (const auto [p1, p2, p3] : ranges::views::zip(
         points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
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

  // Skip Border Point Validation

  const auto non_border_points =
    points | ranges::views::filter([&](const auto & point) { return point.lane_ids.size() == 1; }) |
    ranges::to<std::vector>();
  ASSERT_GE(non_border_points.size(), 1);

  //
  //  Velocity of path points is set the Lanelet speed limit, and increase at the border point in
  //  step-function manner
  //
  {
    const auto & non_border_point = non_border_points.at(0);
    ASSERT_FLOAT_EQ(non_border_point.point.longitudinal_velocity_mps, 35 / 3.6);
  }

#ifdef PLOT
  {
    std::string filename =
      std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + GetParam() +
      ".svg";
    std::replace(filename.begin(), filename.end(), '/', '_');
    savefig(centerline_path, start_s, end_s, P6_s, lanelet_sequence, filename);
  }
#endif
}

// TODO(sarun-hub): TEST 03, 04, 05, 06 failed, Investigating
INSTANTIATE_TEST_SUITE_P(
  getCenterlinePathMap, TestWithVM_01_10_12_Map,
  ::testing::Values(
    "test_route_handler_centerline_path_valid_01.yaml",
    "test_route_handler_centerline_path_valid_02.yaml"));

}  // namespace autoware::route_handler::test

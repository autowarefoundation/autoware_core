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

#include "test_route_handler.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

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

template <typename Parameter>
class TestCase : public TestRouteHandler, public ::testing::WithParamInterface<Parameter>
{
protected:
  void SetUp() override
  {
    using autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map;
    const auto map_rel_path = static_cast<std::string>(Parameter::dir) + "/" + "lanelet2_map.osm";
    const auto map_abs_path =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map" / map_rel_path;
    std::cout << "Load map from " << map_abs_path << std::endl;
    const auto lanelet_map_ = load_mgrs_coordinate_map(map_abs_path);
    route_handler_ = std::make_shared<RouteHandler>(lanelet_map_);
  }

  std::shared_ptr<RouteHandler> route_handler_;
};

struct Parameter_Map_Waypoint_Straight_00  // NOLINT
{
  static constexpr const char * dir = "vm_01_10-12/straight_waypoint";
  const std::vector<lanelet::Id> route_lane_ids;
  const double start_s;
  const double end_s;
  const bool expect_success;
  const double expected_approximate_length_lower_bound;
};

using TestCase_Map_Waypoint_Straight_00 = TestCase<Parameter_Map_Waypoint_Straight_00>;  // NOLINT

TEST_P(TestCase_Map_Waypoint_Straight_00, test_path_validity)
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;
  using autoware::experimental::lanelet2_utils::from_ros;
  auto [ids, start_s, end_s, expect_success, expected_approximate_length_lower_bound] = GetParam();

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  if (expect_success) {
    EXPECT_TRUE(
      calculate_path_with_lane_id_length(centerline_path) > expected_approximate_length_lower_bound)
      << "length of centerline_path / expected_approximate_length_lower_bound = "
      << calculate_path_with_lane_id_length(centerline_path) << ", "
      << expected_approximate_length_lower_bound;

    const auto points = centerline_path.points;
    const auto lanelet_opt = combine_lanelets_shape(lanelet_sequence);

    if (lanelet_opt.has_value()) {
      const auto lanelet = lanelet_opt.value();

      for (const auto [p1, p2, p3] : ranges::views::zip(
             points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
        EXPECT_TRUE(
          autoware_utils_geometry::calc_distance3d(p1, p2) >=
          autoware::experimental::trajectory::k_points_minimum_dist_threshold);
        EXPECT_TRUE(
          boost::geometry::within(
            lanelet::utils::to2D(from_ros(p2.point.pose.position)),
            lanelet.polygon2d().basicPolygon()))
          << "point(" << p2.point.pose.position.x << ", " << p2.point.pose.position.y << ")";
        EXPECT_TRUE(
          std::fabs(
            autoware_utils_math::normalize_radian(
              autoware_utils_geometry::calc_azimuth_angle(
                p1.point.pose.position, p2.point.pose.position) -
              autoware_utils_geometry::calc_azimuth_angle(
                p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
      }
    }
  } else {
    // minimum size if 2 in case that start_s == end_s.
    ASSERT_EQ(centerline_path.points.size(), 2);
    // there is no handler for end_s < start_s, so at least there will be 2 points.
  }
}

// straight_waypoint/lanelet2_map.osm
// start point: (102, 100) -> go straight upward

INSTANTIATE_TEST_SUITE_P(
  test_path_validity, TestCase_Map_Waypoint_Straight_00,
  ::testing::Values(  // enumerate values below
    Parameter_Map_Waypoint_Straight_00{
      {1043, 1047, 1049},  // ids
      200.0,               // start_s[m]
      400.0,               // end_s[m]
      true,                // expect_success
      200 * 0.9            // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Straight_00{
      {1043, 1047, 1049},  // ids
      100.0,               // start_s[m]
      400.0,               // end_s[m]
      true,                // expect_success
      200 * 0.9            // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Straight_00{
      {1043, 1047, 1049},  // ids
      200.0,               // start_s[m]
      200.0,               // end_s[m]
      false,               // expect_success
      200 * 0.9            // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Straight_00{
      {1043, 1047, 1049},  // ids
      -inf,                // start_s[m]
      inf,                 // end_s[m]
      true,                // expect_success
      200 * 0.9            // length_lower_bound[m]
    }));

struct Parameter_Map_Waypoint_Curve_00  // NOLINT
{
  static constexpr const char * dir = "vm_01_10-12/dense_centerline";
  const std::vector<lanelet::Id> route_lane_ids;
  const double start_s;
  const double end_s;
  const bool expect_success;
  const double expected_approximate_length_lower_bound;
};

using TestCase_Map_Waypoint_Curve_00 = TestCase<Parameter_Map_Waypoint_Curve_00>;  // NOLINT

TEST_P(TestCase_Map_Waypoint_Curve_00, test_path_validity)
{
  using autoware::experimental::lanelet2_utils::combine_lanelets_shape;
  using autoware::experimental::lanelet2_utils::from_ros;
  auto [ids, start_s, end_s, expect_success, expected_approximate_length_lower_bound] = GetParam();

  const auto lanelet_sequence = route_handler_->getLaneletsFromIds(ids);

  const auto centerline_path = route_handler_->getCenterLinePath(lanelet_sequence, start_s, end_s);

  if (expect_success) {
    EXPECT_TRUE(
      calculate_path_with_lane_id_length(centerline_path) > expected_approximate_length_lower_bound)
      << "length of centerline_path / expected_approximate_length_lower_bound = "
      << calculate_path_with_lane_id_length(centerline_path) << ", "
      << expected_approximate_length_lower_bound;

    const auto points = centerline_path.points;
    const auto lanelet_opt = combine_lanelets_shape(lanelet_sequence);

    if (lanelet_opt.has_value()) {
      const auto lanelet = lanelet_opt.value();

      for (const auto [p1, p2, p3] : ranges::views::zip(
             points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
        EXPECT_TRUE(
          autoware_utils_geometry::calc_distance3d(p1, p2) >=
          autoware::experimental::trajectory::k_points_minimum_dist_threshold);
        EXPECT_TRUE(
          boost::geometry::within(
            lanelet::utils::to2D(from_ros(p2.point.pose.position)),
            lanelet.polygon2d().basicPolygon()))
          << "point(" << p2.point.pose.position.x << ", " << p2.point.pose.position.y << ")";
        EXPECT_TRUE(
          std::fabs(
            autoware_utils_math::normalize_radian(
              autoware_utils_geometry::calc_azimuth_angle(
                p1.point.pose.position, p2.point.pose.position) -
              autoware_utils_geometry::calc_azimuth_angle(
                p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
      }
    }
  } else {
    // minimum size if 2 in case that start_s == end_s.
    ASSERT_EQ(centerline_path.points.size(), 2);
    // there is no handler for end_s < start_s, so at least there will be 2 points.
  }
}

INSTANTIATE_TEST_SUITE_P(
  test_path_validity, TestCase_Map_Waypoint_Curve_00,
  ::testing::Values(  // enumerate values below
    Parameter_Map_Waypoint_Curve_00{
      {140, 137, 136, 138, 139, 135},  // ids
      10.0,                // start_s[m] 10
      50.0,                 // end_s[m] 10 + 40
      true,                 // expect_success
      40 * 0.9              // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Curve_00{
      {140, 137, 136, 138, 139, 135},  // ids
      3.5,                 // start_s[m] 10 - 6.5
      10.0,                 // end_s[m] 10
      true,                 // expect_success
      6.5 * 0.9             // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Curve_00{
      {140, 137, 136, 138, 139, 135},  // ids
      3.5,                // start_s[m] 10 - 6.5
      50.0,                 // end_s[m] 10 + 40
      true,                 // expect_success
      45 * 0.9              // length_lower_bound[m]
    },
    Parameter_Map_Waypoint_Curve_00{
      {140, 137, 136, 138, 139, 135},  // ids
      10.0,                 // start_s[m]
      10.0,                 // end_s[m]
      false,                // expect_success
      0.0                   // length_lower_bound[m]
    },
  Parameter_Map_Waypoint_Curve_00{
      {140, 137, 136, 138, 139, 135},  // ids
      -inf,                 // start_s[m]
      inf,                  // end_s[m]
      true,                 // expect_success
      40 * 0.9              // length_lower_bound[m]
    }));
}  // namespace autoware::route_handler::test

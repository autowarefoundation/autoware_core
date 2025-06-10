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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>

#include <array>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr auto inf = std::numeric_limits<double>::infinity();

namespace autoware::experimental
{

template <typename Parameter>
class TestCase : public ::testing::TestWithParam<Parameter>
{
public:
protected:
  lanelet::LaneletMapConstPtr lanelet_map_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory(Parameter::pkg)) / "sample_map" /
      std::string(Parameter::dir);
    const auto map_path = sample_map_dir / "lanelet2_map.osm";
    lanelet_map_ = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
    std::tie(routing_graph_, traffic_rules_) =
      autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
        lanelet_map_);
  }
};

struct Parameter_Map_Waypoint_Straight_00  // NOLINT
{
  static constexpr const char * pkg = "autoware_lanelet2_utils";
  static constexpr const char * dir = "straight_waypoint";
  const double forward_length;
  const double backward_length;
  const std::vector<lanelet::Id> route_lane_ids;
  const lanelet::Id current_lane_id;
  const double ego_x;
  const double ego_y;
  const double ego_z;
  const std::array<double, 4> ego_quat;
  const bool expect_success;
  const double expected_approximate_length_lower_bound;
};

void PrintTo(const Parameter_Map_Waypoint_Straight_00 & param, ::std::ostream * os)  // NOLINT
{
  *os << "forward/backward length = (" << param.forward_length << ", " << param.backward_length
      << "), (x, y, z) = (" << param.ego_x << ", " << param.ego_y << ", " << param.ego_z << ")";
}

using TestCase_Map_Waypoint_Straight_00 = TestCase<Parameter_Map_Waypoint_Straight_00>;  // NOLINT

TEST_P(TestCase_Map_Waypoint_Straight_00, test_path_validity)
{
  auto
    [FORWARD_LENGTH, BACKWARD_LENGTH, ids, current_id, x, y, z, quat, expect_success,
     expected_approximate_length_lower_bound] = GetParam();

  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(x, y, z))
      .orientation(autoware_utils_geometry::create_quaternion(quat[0], quat[1], quat[2], quat[3]));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_map_->laneletLayer.get(current_id), ego_pose, lanelet_map_,
    routing_graph_, traffic_rules_, FORWARD_LENGTH, BACKWARD_LENGTH);

  if (expect_success) {
    ASSERT_TRUE(reference_path_opt.has_value());
    const auto & reference_path = reference_path_opt.value();
    EXPECT_TRUE(reference_path.length() > expected_approximate_length_lower_bound)
      << "length of reference_path / expected_approximate_length_lower_bound = "
      << reference_path.length() << ", " << expected_approximate_length_lower_bound;
    /*
    // TODO(soblin): if waypoints are contained, trajectory length does not match with s_end -
    // s_start
    if (expect_length.value() != inf) {
      EXPECT_TRUE(std::fabs(reference_path.length() - expect_length.value()) < 0.1)
        << "length of reference_path / expected = " << reference_path.length() << ", "
        << expect_length.value();
    }
    */

    const auto points = reference_path.restore();
    const auto lanelet = lanelet::utils::combineLaneletsShape(lanelet_sequence);

    for (const auto [p1, p2, p3] : ranges::views::zip(
           points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
      EXPECT_TRUE(
        autoware_utils_geometry::calc_distance3d(p1, p2) >=
        autoware::experimental::trajectory::k_points_minimum_dist_threshold);
      EXPECT_TRUE(boost::geometry::within(
        lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(p2.point.pose.position)),
        lanelet.polygon2d().basicPolygon()))
        << "point(" << p2.point.pose.position.x << ", " << p2.point.pose.position.y << ")";
      EXPECT_TRUE(
        std::fabs(autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
    }
  } else {
    ASSERT_FALSE(reference_path_opt.has_value());
  }
}

INSTANTIATE_TEST_SUITE_P(
  test_path_validity, TestCase_Map_Waypoint_Straight_00,
  ::testing::Values(  // enumerate values below
    Parameter_Map_Waypoint_Straight_00{
      200,                   // [m]
      0,                     // [m]
      {1043, 1047, 1049},    // ids
      1043,                  // id
      102,                   // x[m]
      300,                   // y[m]
      100.0,                 // z[m]
      {0.0, 0.0, 0.0, 1.0},  // quaternion
      true,
      200 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Straight_00{
      200,                   // [m]
      100,                   // [m]
      {1043, 1047, 1049},    // ids
      1043,                  // id
      102,                   // x[m]
      300,                   // y[m]
      100.0,                 // z[m]
      {0.0, 0.0, 0.0, 1.0},  // quaternion
      true,
      200 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Straight_00{
      0,                     // [m]
      0,                     // [m]
      {1043, 1047, 1049},    // ids
      1043,                  // id
      102,                   // x[m]
      300,                   // y[m]
      100.0,                 // z[m]
      {0.0, 0.0, 0.0, 1.0},  // quaternion
      false,
      0.0  // [m]
    },
    Parameter_Map_Waypoint_Straight_00{
      inf,                   // [m]
      inf,                   // [m]
      {1043, 1047, 1049},    // ids
      1043,                  // id
      102,                   // x[m]
      300,                   // y[m]
      100.0,                 // z[m]
      {0.0, 0.0, 0.0, 1.0},  // quaternion
      true,
      200 * 0.9  // [m]
    })           // values
);

struct Parameter_Map_Waypoint_Curve_00  // NOLINT
{
  static constexpr const char * pkg = "autoware_lanelet2_utils";
  static constexpr const char * dir = "dense_centerline";
  const double forward_length;
  const double backward_length;
  const std::vector<lanelet::Id> route_lane_ids;
  const lanelet::Id current_lane_id;
  const double ego_x;
  const double ego_y;
  const double ego_z;
  const std::array<double, 4> ego_quat;
  const bool expect_success;
  const double expected_approximate_length_lower_bound;
};

void PrintTo(const Parameter_Map_Waypoint_Curve_00 & param, ::std::ostream * os)  // NOLINT
{
  *os << "forward/backward length = (" << param.forward_length << ", " << param.backward_length
      << "), (x, y, z) = (" << param.ego_x << ", " << param.ego_y << ", " << param.ego_z << ")";
}

using TestCase_Map_Waypoint_Curve_00 = TestCase<Parameter_Map_Waypoint_Curve_00>;  // NOLINT

TEST_P(TestCase_Map_Waypoint_Curve_00, test_path_validity)
{
  auto
    [FORWARD_LENGTH, BACKWARD_LENGTH, ids, current_id, x, y, z, quat, expect_success,
     expected_approximate_length_lower_bound] = GetParam();

  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(x, y, z))
      .orientation(autoware_utils_geometry::create_quaternion(quat[0], quat[1], quat[2], quat[3]));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_map_->laneletLayer.get(current_id), ego_pose, lanelet_map_,
    routing_graph_, traffic_rules_, FORWARD_LENGTH, BACKWARD_LENGTH);

  if (expect_success) {
    ASSERT_TRUE(reference_path_opt.has_value());
    const auto & reference_path = reference_path_opt.value();
    EXPECT_TRUE(reference_path.length() > expected_approximate_length_lower_bound)
      << "length of reference_path / expected_approximate_length_lower_bound = "
      << reference_path.length() << ", " << expected_approximate_length_lower_bound;
    /*
    // TODO(soblin): if waypoints are contained, trajectory length does not match with s_end -
    // s_start
    if (expect_length.value() != inf) {
      EXPECT_TRUE(std::fabs(reference_path.length() - expect_length.value()) < 0.1)
        << "length of reference_path / expected = " << reference_path.length() << ", "
        << expect_length.value();
    }
    */

    const auto points = reference_path.restore();
    const auto lanelet = lanelet::utils::combineLaneletsShape(lanelet_sequence);

    for (const auto [p1, p2, p3] : ranges::views::zip(
           points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
      EXPECT_TRUE(
        autoware_utils_geometry::calc_distance3d(p1, p2) >=
        autoware::experimental::trajectory::k_points_minimum_dist_threshold);

      // use p2, because p1/p3 at the end may well be slightly outside of the Lanelets by error
      EXPECT_TRUE(boost::geometry::within(
        lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(p2.point.pose.position)),
        lanelet.polygon2d().basicPolygon()))
        << "point(" << p2.point.pose.position.x << ", " << p2.point.pose.position.y << ")";

      EXPECT_TRUE(
        std::fabs(autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
    }
  } else {
    ASSERT_FALSE(reference_path_opt.has_value());
  }
}

INSTANTIATE_TEST_SUITE_P(
  test_path_validity, TestCase_Map_Waypoint_Curve_00,
  ::testing::Values(  // enumerate values below
                      // here are the cases where current_pose is on a normal lanelet
    Parameter_Map_Waypoint_Curve_00{
      40,                              // [m]
      0,                               // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      140,                             // id
      740,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      40 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      0,                               // [m]
      6.5,                             // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      140,                             // id
      740,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      6.5 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      40,                              // [m]
      6.5,                             // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      140,                             // id
      740,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      45 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      0,                               // [m]
      0,                               // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      140,                             // id
      740,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      false,
      0.0  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      inf,                             // [m]
      inf,                             // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      140,                             // id
      740,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      50 * 0.9  // [m]
    },
    // here are the cases where current_pose is just before a waypoint
    Parameter_Map_Waypoint_Curve_00{
      40,                              // [m]
      0,                               // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      137,                             // id
      735,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      40 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      0,                               // [m]
      6.5,                             // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      137,                             // id
      735,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      6.5 * 0.9  // [m]
    },
    Parameter_Map_Waypoint_Curve_00{
      40,                              // [m]
      6.5,                             // [m]
      {140, 137, 136, 138, 139, 135},  // ids
      137,                             // id
      735,                             // x[m]
      1148,                            // y[m]
      100.0,                           // z[m]
      {0.0, 0.0, 0.0, 1.0},            // quaternion
      true,
      45 * 0.9  // [m]
    })          // values
);

struct Parameter_Map_Overlap_Lane_00  // NOLINT
{
  static constexpr const char * pkg = "autoware_test_utils";
  static constexpr const char * dir = "overlap";
  const double forward_length;
  const double backward_length;
  const std::vector<lanelet::Id> route_lane_ids;
  const lanelet::Id current_lane_id;
  const double ego_x;
  const double ego_y;
  const double ego_z;
  const std::array<double, 4> ego_quat;
  const bool expect_success;
};

void PrintTo(const Parameter_Map_Overlap_Lane_00 & param, ::std::ostream * os)  // NOLINT
{
  *os << "forward/backward length = (" << param.forward_length << ", " << param.backward_length
      << "), (x, y, z) = (" << param.ego_x << ", " << param.ego_y << ", " << param.ego_z << ")";
}

class TestCase_Map_Overlap_Lane_00 : public ::testing::TestWithParam<Parameter_Map_Overlap_Lane_00>
{
public:
protected:
  lanelet::LaneletMapConstPtr lanelet_map_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_{nullptr};

  void SetUp() override
  {
    // TODO(soblin): unify test map location and PATHs! (one of autoware_test_utils,
    // autoware_lanelet2_utils)
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory(Parameter_Map_Overlap_Lane_00::pkg)) /
      "test_map" / std::string(Parameter_Map_Overlap_Lane_00::dir);
    const auto map_path = sample_map_dir / "lanelet2_map.osm";
    lanelet_map_ = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
    std::tie(routing_graph_, traffic_rules_) =
      autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
        lanelet_map_);
  }
};

TEST_P(TestCase_Map_Overlap_Lane_00, test_path_validity)
{
  auto [FORWARD_LENGTH, BACKWARD_LENGTH, ids, current_id, x, y, z, quat, expect_success] =
    GetParam();

  const auto lanelet_sequence =
    ids |
    ranges::views::transform([&](const auto & id) { return lanelet_map_->laneletLayer.get(id); }) |
    ranges::to<std::vector>();
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(x, y, z))
      .orientation(autoware_utils_geometry::create_quaternion(quat[0], quat[1], quat[2], quat[3]));
  const auto reference_path_opt = trajectory::build_reference_path(
    lanelet_sequence, lanelet_map_->laneletLayer.get(current_id), ego_pose, lanelet_map_,
    routing_graph_, traffic_rules_, FORWARD_LENGTH, BACKWARD_LENGTH);

  if (expect_success) {
    ASSERT_TRUE(reference_path_opt.has_value());
    const auto & reference_path = reference_path_opt.value();
    // NOTE(soblin): this map does not have waypoint, so s_end - s_start matches the trajectory
    // length
    const double expect_length = FORWARD_LENGTH + BACKWARD_LENGTH;
    EXPECT_TRUE(std::fabs(reference_path.length() - expect_length) < 0.1)
      << "length of reference_path / expected = " << reference_path.length() << ", "
      << expect_length;

    const auto points = reference_path.restore();
    const auto lanelet = lanelet::utils::combineLaneletsShape(lanelet_sequence);

    for (const auto [p1, p2, p3] : ranges::views::zip(
           points, points | ranges::views::drop(1), points | ranges::views::drop(2))) {
      EXPECT_TRUE(
        autoware_utils_geometry::calc_distance3d(p1, p2) >=
        autoware::experimental::trajectory::k_points_minimum_dist_threshold);

      // use p2, because p1/p3 at the end may well be slightly outside of the Lanelets by error
      EXPECT_TRUE(boost::geometry::within(
        lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(p2.point.pose.position)),
        lanelet.polygon2d().basicPolygon()))
        << "point(" << p2.point.pose.position.x << ", " << p2.point.pose.position.y << ")";

      EXPECT_TRUE(
        std::fabs(autoware_utils_math::normalize_radian(
          autoware_utils_geometry::calc_azimuth_angle(
            p1.point.pose.position, p2.point.pose.position) -
          autoware_utils_geometry::calc_azimuth_angle(
            p2.point.pose.position, p3.point.pose.position))) < M_PI / 2.0);
    }
  } else {
    ASSERT_FALSE(reference_path_opt.has_value());
  }
}

}  // namespace autoware::experimental

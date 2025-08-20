// Copyright 2024 TIER IV, Inc.
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
#include "autoware/trajectory/utils/reference_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/visualization.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace autoware::experimental;  // NOLINT

namespace
{
std::optional<trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
build_reference_path(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & current_lanelet,
  const geometry_msgs::msg::Pose & ego_pose, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules, const double forward_length,
  const double backward_length, const double waypoint_connection_gradient_from_centerline)
{
  const auto s_ego = autoware::experimental::trajectory::get_position_in_lanelet_sequence(
    autoware::experimental::trajectory::zip_accumulated_distance(lanelet_sequence),
    autoware::experimental::trajectory::Waypoint{
      lanelet::utils::conversion::toLaneletPoint(ego_pose.position), current_lanelet.id()});
  if (!s_ego) {
    // ego is not in lanelet sequence
    return std::nullopt;
  }

  const auto extended_lanelet_sequence_with_interval =
    autoware::experimental::trajectory::extend_lanelet_sequence(
      lanelet_sequence, routing_graph, {*s_ego - backward_length, *s_ego + forward_length});

  if (
    extended_lanelet_sequence_with_interval.interval.end <=
    extended_lanelet_sequence_with_interval.interval.start) {
    // interval is invalid
    return std::nullopt;
  }

  return autoware::experimental::trajectory::build_reference_path(
    extended_lanelet_sequence_with_interval, lanelet_map, traffic_rules,
    waypoint_connection_gradient_from_centerline);
}
}  // namespace

int main1()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map/vm_01_10-12/dense_centerline";
  const std::vector<lanelet::Id> ids = {140, 137, 136, 138, 139, 135};
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(108, 99, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));
  const auto map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(138);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    auto & ax = axes[0];
    const double forward_length = 40;
    const double backward_length = 0.0;
    const double waypoint_connection_gradient_from_centerline = 10.0;
    const auto reference_path_opt = build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length, waypoint_connection_gradient_from_centerline);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 40, backward = 0 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

int main2()
{
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);

  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_test_utils")) /
    "test_map/overlap";
  const std::vector<lanelet::Id> ids = {609, 610, 612, 611, 613};
  const auto ego_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(autoware_utils_geometry::create_point(1573.68, 389.857, 100.0))
      .orientation(autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.999997, 0.00250111));
  const auto map_path = sample_map_dir / "lanelet2_map.osm";

  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  const auto [routing_graph, traffic_rules] =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr);
  const auto current_lanelet = lanelet_map_ptr->laneletLayer.get(609);

  const auto lanelet_sequence = ids | ranges::views::transform([&](const auto & id) {
                                  return lanelet_map_ptr->laneletLayer.get(id);
                                }) |
                                ranges::to<std::vector>();
  auto path_plot_config = autoware::test_utils::PathWithLaneIdConfig::defaults();
  path_plot_config.linewidth = 2.0;
  path_plot_config.color = "orange";
  path_plot_config.quiver_size = 2.0;

  auto lane_plot_config = autoware::test_utils::LaneConfig::defaults();
  lane_plot_config.line_config = autoware::test_utils::LineConfig::defaults();

  {
    const double forward_length = 50;
    const double backward_length = 10.0;
    const double waypoint_connection_gradient_from_centerline = 10.0;
    auto & ax = axes[0];
    const auto reference_path_opt = build_reference_path(
      lanelet_sequence, current_lanelet, ego_pose, lanelet_map_ptr, routing_graph, traffic_rules,
      forward_length, backward_length, waypoint_connection_gradient_from_centerline);
    if (reference_path_opt) {
      const auto & reference_path = reference_path_opt.value();
      autoware_internal_planning_msgs::msg::PathWithLaneId path;
      path.points = reference_path.restore();
      autoware::test_utils::plot_autoware_object(path, ax, path_plot_config);
      ax.set_title(Args(
        "forward = 50, backward = 10 (actual length = " + std::to_string(reference_path.length()) +
        ")"));
    }
    for (const auto & route_lanelet : lanelet_sequence) {
      autoware::test_utils::plot_lanelet2_object(route_lanelet, ax, lane_plot_config);
    }
    ax.scatter(Args(ego_pose.position.x, ego_pose.position.y), Kwargs("label"_a = "ego position"));
    ax.set_aspect(Args("equal"));
    ax.legend();
    ax.grid();
  }
  plt.show();

  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  main1();
  main2();
}
